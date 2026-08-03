/*
 * Copyright (c) 2024 Andri Yngvason
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#include "enc/h264-encoder.h"
#include "neatvnc.h"
#include "frame.h"
#include "pixels.h"

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>
#include <drm_fourcc.h>
#include <gbm.h>
#include <aml.h>
#include <dirent.h>
#include <errno.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>

#define UDIV_UP(a, b) (((a) + (b) - 1) / (b))
#define ALIGN_UP(a, b) ((b) * UDIV_UP((a), (b)))
#define ARRAY_LENGTH(a) (sizeof(a) / sizeof((a)[0]))

#define N_SRC_BUFS 3
#define N_DST_BUFS 3

enum v4l2m2m_src_memory {
	V4L2M2M_SRC_MEMORY_DMABUF,
	V4L2M2M_SRC_MEMORY_MMAP,
};

struct h264_encoder_v4l2m2m_dst_buf {
	struct v4l2_buffer buffer;
	struct v4l2_plane plane;
	void* payload;
};

struct h264_encoder_v4l2m2m_src_buf {
	struct v4l2_buffer buffer;
	struct v4l2_plane planes[4];
	int fd;
	bool is_taken;
	struct nvnc_frame* fb;

	/* Only used when source memory is MMAP */
	void* mmap_payload[4];
	uint32_t mmap_stride[4];
};

struct h264_encoder_v4l2m2m {
	struct h264_encoder base;

	uint32_t width;
	uint32_t height;
	uint32_t format;
	int quality; // TODO: Can we affect the quality?

	char driver[16];

	int fd;
	struct aml_handler* handler;

	uint32_t src_type;
	uint32_t dst_type;
	uint32_t src_pixfmt;
	enum v4l2m2m_src_memory src_memory;

	/* Only used when source memory is MMAP (software conversion) */
	struct SwsContext* sws_ctx;
	enum AVPixelFormat src_av_format;

	struct h264_encoder_v4l2m2m_src_buf src_bufs[N_SRC_BUFS];
	int src_buf_index;

	struct h264_encoder_v4l2m2m_dst_buf dst_bufs[N_DST_BUFS];
};

struct h264_encoder_impl h264_encoder_v4l2m2m_impl;

bool h264_encoder_v4l2m2m_probe(uint32_t width, uint32_t height,
		uint32_t format);

static int v4l2_qbuf(int fd, const struct v4l2_buffer* inbuf)
{
	assert(inbuf->length <= 4);
	struct v4l2_plane planes[4];
	struct v4l2_buffer outbuf;
	outbuf = *inbuf;
	memcpy(&planes, inbuf->m.planes, inbuf->length * sizeof(planes[0]));
	outbuf.m.planes = planes;
	return ioctl(fd, VIDIOC_QBUF, &outbuf);
}

static inline int v4l2_dqbuf(int fd, struct v4l2_buffer* buf)
{
	return ioctl(fd, VIDIOC_DQBUF, buf);
}

static struct h264_encoder_v4l2m2m_src_buf* take_src_buffer(
		struct h264_encoder_v4l2m2m* self)
{
	unsigned int count = 0;
	int i = self->src_buf_index;

	struct h264_encoder_v4l2m2m_src_buf* buffer;
	do {
		buffer = &self->src_bufs[i++];
		i %= ARRAY_LENGTH(self->src_bufs);
	} while (++count < ARRAY_LENGTH(self->src_bufs) && buffer->is_taken);

	if (buffer->is_taken)
		return NULL;

	self->src_buf_index = i;
	buffer->is_taken = true;

	return buffer;
}

static bool any_src_buf_is_taken(struct h264_encoder_v4l2m2m* self)
{
	bool result = false;
	for (unsigned int i = 0; i < ARRAY_LENGTH(self->src_bufs); ++i)
		if (self->src_bufs[i].is_taken)
			result = true;
	return result;
}

static int u32_cmp(const void* pa, const void* pb)
{
	const uint32_t* a = pa;
	const uint32_t* b = pb;
	return *a < *b ? -1 : *a > *b;
}

static size_t get_supported_formats(struct h264_encoder_v4l2m2m* self,
		uint32_t type, uint32_t* formats, size_t max_len)
{
	size_t i = 0;
	for (;; ++i) {
		struct v4l2_fmtdesc desc = {
			.index = i,
			.type = type,
		};
		int rc = ioctl(self->fd, VIDIOC_ENUM_FMT, &desc);
		if (rc < 0)
			break;

		if (i >= max_len)
			break;

		nvnc_trace("Got pixel format: %s", desc.description);

		formats[i] = desc.pixelformat;
	}

	qsort(formats, i, sizeof(*formats), u32_cmp);

	return i;
}

static bool have_v4l2_format(const uint32_t* formats, size_t n_formats,
		uint32_t format)
{
	return bsearch(&format, formats, n_formats, sizeof(format), u32_cmp);
}

static uint32_t v4l2_format_from_drm(const uint32_t* formats,
		size_t n_formats, uint32_t drm_format)
{
#define TRY_FORMAT(f) \
		if (have_v4l2_format(formats, n_formats, f)) \
			return f

	switch (drm_format) {
	case DRM_FORMAT_NV12:
		TRY_FORMAT(V4L2_PIX_FMT_NV12);
		break;
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_RGBA8888:
		TRY_FORMAT(V4L2_PIX_FMT_RGBX32);
		TRY_FORMAT(V4L2_PIX_FMT_RGBA32);
		break;
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		TRY_FORMAT(V4L2_PIX_FMT_XRGB32);
		TRY_FORMAT(V4L2_PIX_FMT_ARGB32);
		TRY_FORMAT(V4L2_PIX_FMT_RGB32);
		break;
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_BGRA8888:
		TRY_FORMAT(V4L2_PIX_FMT_XBGR32);
		TRY_FORMAT(V4L2_PIX_FMT_ABGR32);
		TRY_FORMAT(V4L2_PIX_FMT_BGR32);
		break;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		TRY_FORMAT(V4L2_PIX_FMT_BGRX32);
		TRY_FORMAT(V4L2_PIX_FMT_BGRA32);
		break;
	// TODO: More formats
	}

	return 0;
#undef TRY_FORMAT
}

static enum AVPixelFormat drm_format_to_av_pixel_format(uint32_t drm_format)
{
	switch (drm_format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return AV_PIX_FMT_BGR0;
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_ABGR8888:
		return AV_PIX_FMT_RGB0;
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_RGBA8888:
		return AV_PIX_FMT_0BGR;
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_BGRA8888:
		return AV_PIX_FMT_0RGB;
	}

	return AV_PIX_FMT_NONE;
}

// This driver mixes up pixel formats...
static uint32_t v4l2_format_from_drm_bcm2835(const uint32_t* formats,
		size_t n_formats, uint32_t drm_format)
{
	switch (drm_format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return V4L2_PIX_FMT_RGBA32;
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_BGRA8888:
		// TODO: This could also be ABGR, based on how this driver
		// behaves
		return V4L2_PIX_FMT_BGR32;
	}
	return 0;
}

static int select_src_format(struct h264_encoder_v4l2m2m* self)
{
	uint32_t supported_formats[256];
	size_t n_formats = get_supported_formats(self, self->src_type,
			supported_formats, ARRAY_LENGTH(supported_formats));

	uint32_t direct_format;
	if (strcmp(self->driver, "bcm2835-codec") == 0)
		direct_format = v4l2_format_from_drm_bcm2835(supported_formats,
				n_formats, self->format);
	else
		direct_format = v4l2_format_from_drm(supported_formats, n_formats,
				self->format);

	if (direct_format) {
		self->src_pixfmt = direct_format;
		self->src_memory = V4L2M2M_SRC_MEMORY_DMABUF;
		self->src_av_format = AV_PIX_FMT_NONE;
		return 0;
	}

	/* Fall back to NV12 with a CPU colour-space conversion.  This is needed
	 * for drivers such as qcom-iris that only accept YUV input.
	 */
	if (!have_v4l2_format(supported_formats, n_formats, V4L2_PIX_FMT_NV12)) {
		nvnc_log(NVNC_LOG_DEBUG,
				"Failed to find a proper pixel format for v4l2m2m");
		return -1;
	}

	enum AVPixelFormat av_format = drm_format_to_av_pixel_format(self->format);
	if (av_format == AV_PIX_FMT_NONE) {
		nvnc_log(NVNC_LOG_DEBUG,
				"No software conversion from DRM format %.4s to NV12",
				(const char*)&self->format);
		return -1;
	}

	struct SwsContext* sws = sws_getContext(self->width, self->height,
			av_format, self->width, self->height, AV_PIX_FMT_NV12,
			SWS_FAST_BILINEAR, NULL, NULL, NULL);
	if (!sws) {
		nvnc_log(NVNC_LOG_DEBUG,
				"Failed to create swscale context for NV12 conversion");
		return -1;
	}

	self->src_pixfmt = V4L2_PIX_FMT_NV12;
	self->src_memory = V4L2M2M_SRC_MEMORY_MMAP;
	self->src_av_format = av_format;
	self->sws_ctx = sws;
	return 0;
}

static int set_src_fmt(struct h264_encoder_v4l2m2m* self)
{
	int rc;

	if (select_src_format(self) < 0)
		return -1;

	struct v4l2_format fmt = {
		.type = self->src_type,
	};
	rc = ioctl(self->fd, VIDIOC_G_FMT, &fmt);
	if (rc < 0)
		return -1;

	struct v4l2_pix_format_mplane* pix_fmt = &fmt.fmt.pix_mp;
	pix_fmt->pixelformat = self->src_pixfmt;

	if (strcmp(self->driver, "bcm2835-codec") == 0) {
		pix_fmt->width = ALIGN_UP(self->width, 16);
		pix_fmt->height = ALIGN_UP(self->height, 16);
	} else {
		pix_fmt->width = self->width;
		pix_fmt->height = self->height;
	}

	rc = ioctl(self->fd, VIDIOC_S_FMT, &fmt);
	if (rc < 0)
		return -1;

	if (pix_fmt->width != self->width || pix_fmt->height != self->height) {
		struct v4l2_selection sel = {
			.type = self->src_type,
			.target = V4L2_SEL_TGT_CROP,
			.r = {
				.width = self->width,
				.height = self->height,
			},
		};
		ioctl(self->fd, VIDIOC_S_SELECTION, &sel);
	}

	return 0;
}

static int set_dst_fmt(struct h264_encoder_v4l2m2m* self)
{
	int rc;

	struct v4l2_format fmt = {
		.type = self->dst_type,
	};
	rc = ioctl(self->fd, VIDIOC_G_FMT, &fmt);
	if (rc < 0)
		return -1;

	struct v4l2_pix_format_mplane* pix_fmt = &fmt.fmt.pix_mp;
	pix_fmt->pixelformat = V4L2_PIX_FMT_H264;
	pix_fmt->width = self->width;
	pix_fmt->height = self->height;

	rc = ioctl(self->fd, VIDIOC_S_FMT, &fmt);
	if (rc < 0)
		return -1;

	return 0;
}

static int alloc_dst_buffers(struct h264_encoder_v4l2m2m* self)
{
	int n_bufs = ARRAY_LENGTH(self->dst_bufs);
	int rc;

	struct v4l2_requestbuffers req = {
		.memory = V4L2_MEMORY_MMAP,
		.count = n_bufs,
		.type = self->dst_type,
	};
	rc = ioctl(self->fd, VIDIOC_REQBUFS, &req);
	if (rc < 0) {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: alloc_dst_buffers REQBUFS failed: %m");
		return -1;
	}

	for (unsigned int i = 0; i < req.count; ++i) {
		struct h264_encoder_v4l2m2m_dst_buf* buffer = &self->dst_bufs[i];
		struct v4l2_buffer* buf = &buffer->buffer;

		buf->index = i;
		buf->type = self->dst_type;
		buf->memory = V4L2_MEMORY_MMAP;
		buf->length = 1;
		buf->m.planes = &buffer->plane;

		rc = ioctl(self->fd, VIDIOC_QUERYBUF, buf);
		if (rc < 0) {
			nvnc_log(NVNC_LOG_DEBUG,
					"v4l2m2m: alloc_dst_buffers QUERYBUF failed buf=%u: %m",
					i);
			return -1;
		}

		buffer->payload = mmap(0, buffer->plane.length,
				PROT_READ | PROT_WRITE, MAP_SHARED, self->fd,
				buffer->plane.m.mem_offset);
		if (buffer->payload == MAP_FAILED) {
			nvnc_log(NVNC_LOG_ERROR,
					"v4l2m2m: alloc_dst_buffers mmap failed buf=%u len=%zu: %m",
					i, (size_t)buffer->plane.length);
			return -1;
		}
	}

	return 0;
}

static void enqueue_dst_buffers(struct h264_encoder_v4l2m2m* self)
{
	for (unsigned int i = 0; i < ARRAY_LENGTH(self->dst_bufs); ++i) {
		int rc __attribute__((unused));
		rc = v4l2_qbuf(self->fd, &self->dst_bufs[i].buffer);
		assert(rc >= 0);
	}
}

static void process_dst_bufs(struct h264_encoder_v4l2m2m* self)
{
	int rc;
	struct v4l2_plane plane = { 0 };
	struct v4l2_buffer buf = {
		.type = self->dst_type,
		.memory = V4L2_MEMORY_MMAP,
		.length = 1,
		.m.planes = &plane,
	};

	while (true) {
		rc = v4l2_dqbuf(self->fd, &buf);
		if (rc < 0)
			break;

		uint64_t pts = buf.timestamp.tv_sec * UINT64_C(1000000) +
			buf.timestamp.tv_usec;
		struct h264_encoder_v4l2m2m_dst_buf* dstbuf =
			&self->dst_bufs[buf.index];
		size_t size = buf.m.planes[0].bytesused;

		static uint64_t last_pts;
		if (last_pts && last_pts > pts) {
			nvnc_log(NVNC_LOG_ERROR, "pts - last_pts = %"PRIi64,
					(int64_t)pts - (int64_t)last_pts);
		}
		last_pts = pts;

		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: encoded frame (index %d) pts %"PRIu64" size %zu",
				buf.index, pts, size);

		self->base.on_packet_ready(dstbuf->payload, size, pts,
				self->base.userdata);

		v4l2_qbuf(self->fd, &buf);
	}
}

static void process_src_bufs(struct h264_encoder_v4l2m2m* self)
{
	int rc;
	struct v4l2_plane planes[4] = { 0 };
	struct v4l2_buffer buf = {
		.type = self->src_type,
		.memory = self->src_memory == V4L2M2M_SRC_MEMORY_DMABUF
			? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP,
		.length = ARRAY_LENGTH(planes),
		.m.planes = planes,
	};

	while (true) {
		rc = v4l2_dqbuf(self->fd, &buf);
		if (rc < 0)
			break;

		struct h264_encoder_v4l2m2m_src_buf* srcbuf =
			&self->src_bufs[buf.index];
		srcbuf->is_taken = false;


		if (self->src_memory == V4L2M2M_SRC_MEMORY_DMABUF) {
			// TODO: This assumes that there's only one fd
			close(srcbuf->planes[0].m.fd);
		}

		if (srcbuf->fb) {
			nvnc_frame_unmap(srcbuf->fb);
			nvnc_frame_unref(srcbuf->fb);
			srcbuf->fb = NULL;
		}
	}
}

static void stream_off(struct h264_encoder_v4l2m2m* self)
{
	ioctl(self->fd, VIDIOC_STREAMOFF, &self->src_type);
	ioctl(self->fd, VIDIOC_STREAMOFF, &self->dst_type);
}

static void free_dst_buffers(struct h264_encoder_v4l2m2m* self)
{
	for (unsigned int i = 0; i < ARRAY_LENGTH(self->dst_bufs); ++i) {
		struct h264_encoder_v4l2m2m_dst_buf* buf = &self->dst_bufs[i];
		if (buf->payload && buf->payload != MAP_FAILED && buf->plane.length > 0)
			munmap(buf->payload, buf->plane.length);
	}
}

static void free_src_buffers(struct h264_encoder_v4l2m2m* self)
{
	if (self->src_memory != V4L2M2M_SRC_MEMORY_MMAP)
		return;

	for (int i = 0; i < N_SRC_BUFS; ++i) {
		struct h264_encoder_v4l2m2m_src_buf* buf = &self->src_bufs[i];
		for (unsigned int j = 0; j < ARRAY_LENGTH(buf->planes); ++j) {
			if (buf->mmap_payload[j] &&
					buf->mmap_payload[j] != MAP_FAILED &&
					buf->planes[j].length > 0)
				munmap(buf->mmap_payload[j], buf->planes[j].length);
			buf->mmap_payload[j] = NULL;
		}
	}
}

static void release_v4l2_buffers(struct h264_encoder_v4l2m2m* self)
{
	struct v4l2_requestbuffers req = { .count = 0 };

	req.memory = V4L2_MEMORY_MMAP;
	req.type = self->dst_type;
	ioctl(self->fd, VIDIOC_REQBUFS, &req);

	req.memory = self->src_memory == V4L2M2M_SRC_MEMORY_DMABUF
		? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
	req.type = self->src_type;
	ioctl(self->fd, VIDIOC_REQBUFS, &req);
}

static int stream_on(struct h264_encoder_v4l2m2m* self)
{
	ioctl(self->fd, VIDIOC_STREAMON, &self->src_type);
	return ioctl(self->fd, VIDIOC_STREAMON, &self->dst_type);
}

static int alloc_src_buffers_dmabuf(struct h264_encoder_v4l2m2m* self)
{
	int rc;

	struct v4l2_requestbuffers req = {
		.memory = V4L2_MEMORY_DMABUF,
		.count = N_SRC_BUFS,
		.type = self->src_type,
	};
	rc = ioctl(self->fd, VIDIOC_REQBUFS, &req);
	if (rc < 0)
		return -1;

	for (int i = 0; i < N_SRC_BUFS; ++i) {
		struct h264_encoder_v4l2m2m_src_buf* buffer = &self->src_bufs[i];
		struct v4l2_buffer* buf = &buffer->buffer;

		buf->index = i;
		buf->type = self->src_type;
		buf->memory = V4L2_MEMORY_DMABUF;
		buf->length = ARRAY_LENGTH(buffer->planes);
		buf->m.planes = buffer->planes;

		rc = ioctl(self->fd, VIDIOC_QUERYBUF, buf);
		if (rc < 0)
			return -1;
	}

	return 0;
}

static int alloc_src_buffers_mmap(struct h264_encoder_v4l2m2m* self)
{
	int rc;

	struct v4l2_requestbuffers req = {
		.memory = V4L2_MEMORY_MMAP,
		.count = N_SRC_BUFS,
		.type = self->src_type,
	};
	rc = ioctl(self->fd, VIDIOC_REQBUFS, &req);
	if (rc < 0)
		return -1;

	for (int i = 0; i < N_SRC_BUFS; ++i) {
		struct h264_encoder_v4l2m2m_src_buf* buffer = &self->src_bufs[i];
		struct v4l2_buffer* buf = &buffer->buffer;

		buf->index = i;
		buf->type = self->src_type;
		buf->memory = V4L2_MEMORY_MMAP;
		buf->length = ARRAY_LENGTH(buffer->planes);
		buf->m.planes = buffer->planes;

		rc = ioctl(self->fd, VIDIOC_QUERYBUF, buf);
		if (rc < 0)
			return -1;

		int n_planes = 0;
		for (unsigned int j = 0; j < ARRAY_LENGTH(buffer->planes); ++j)
			if (buffer->planes[j].length > 0)
				++n_planes;

		struct v4l2_format fmt = {
			.type = self->src_type,
		};
		bool have_fmt = ioctl(self->fd, VIDIOC_G_FMT, &fmt) == 0;

		for (int j = 0; j < n_planes; ++j) {
			buffer->mmap_payload[j] = mmap(0, buffer->planes[j].length,
					PROT_READ | PROT_WRITE, MAP_SHARED, self->fd,
					buffer->planes[j].m.mem_offset);
			if (buffer->mmap_payload[j] == MAP_FAILED) {
				nvnc_log(NVNC_LOG_ERROR,
						"Failed to mmap source buffer plane: %m");
				return -1;
			}

			if (have_fmt && fmt.fmt.pix_mp.plane_fmt[j].bytesperline)
				buffer->mmap_stride[j] =
						fmt.fmt.pix_mp.plane_fmt[j].bytesperline;
			else
				buffer->mmap_stride[j] =
						buffer->planes[j].length / self->height;
		}

		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: src buf %d mapped %d planes, lengths %zu/%zu/%zu/%zu, "
				"strides %u/%u/%u/%u",
				i, n_planes,
				(size_t)buffer->planes[0].length,
				(size_t)buffer->planes[1].length,
				(size_t)buffer->planes[2].length,
				(size_t)buffer->planes[3].length,
				buffer->mmap_stride[0], buffer->mmap_stride[1],
				buffer->mmap_stride[2], buffer->mmap_stride[3]);
	}

	return 0;
}

static int alloc_src_buffers(struct h264_encoder_v4l2m2m* self)
{
	if (self->src_memory == V4L2M2M_SRC_MEMORY_DMABUF)
		return alloc_src_buffers_dmabuf(self);

	return alloc_src_buffers_mmap(self);
}

static void force_key_frame(struct h264_encoder_v4l2m2m* self)
{
	struct v4l2_control ctrl = { 0 };
	ctrl.id = V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME;
	ctrl.value = 0;
	ioctl(self->fd, VIDIOC_S_CTRL, &ctrl);
}

static uint32_t get_plane_size(uint32_t v4l2_format, uint32_t stride,
		uint32_t height, int plane_index)
{
	switch (v4l2_format) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		if (plane_index == 0)
			return stride * height;
		return stride * (height / 2);
	default:
		return stride * height;
	}
}

static int get_v4l2_plane_count(uint32_t v4l2_format)
{
	switch (v4l2_format) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		return 2;
	default:
		return 1;
	}
}

static void encode_buffer_dmabuf(struct h264_encoder_v4l2m2m* self,
		struct h264_encoder_v4l2m2m_src_buf* srcbuf,
		struct nvnc_frame* fb)
{
	struct gbm_bo* bo = nvnc_frame_get_gbm_bo(fb);

	int n_planes = gbm_bo_get_plane_count(bo);
	int fd = gbm_bo_get_fd(bo);
	uint32_t bo_height = gbm_bo_get_height(bo);

	for (int i = 0; i < n_planes; ++i) {
		uint32_t stride = gbm_bo_get_stride_for_plane(bo, i);
		uint32_t offset = gbm_bo_get_offset(bo, i);

		uint32_t plane_height;
		if (self->src_pixfmt == V4L2_PIX_FMT_NV12 ||
				self->src_pixfmt == V4L2_PIX_FMT_NV21) {
			plane_height = (i == 0) ? bo_height : (bo_height / 2);
		} else {
			plane_height = ALIGN_UP(bo_height, 16);
		}

		uint32_t size = get_plane_size(self->src_pixfmt, stride,
				plane_height, i);

		srcbuf->buffer.m.planes[i].m.fd = fd;
		srcbuf->buffer.m.planes[i].bytesused = size;
		srcbuf->buffer.m.planes[i].length = size;
		srcbuf->buffer.m.planes[i].data_offset = offset;
	}

	srcbuf->buffer.length = n_planes;
}

static void encode_buffer_mmap(struct h264_encoder_v4l2m2m* self,
		struct h264_encoder_v4l2m2m_src_buf* srcbuf,
		struct nvnc_frame* fb)
{
	int n_planes = get_v4l2_plane_count(self->src_pixfmt);
	int src_bpp = nvnc_frame_get_pixel_size(fb);
	int src_stride = src_bpp * fb->stride;

	const uint8_t* src_data[1] = { fb->buffer->addr };
	int src_stride_arr[1] = { src_stride };

	uint8_t* dst_data[4] = { 0 };
	int dst_stride[4] = { 0 };
	for (int i = 0; i < n_planes; ++i) {
		dst_data[i] = srcbuf->mmap_payload[i];
		dst_stride[i] = srcbuf->mmap_stride[i];
	}

	/* Some drivers (e.g. qcom-iris) return NV12/NV21 as a single
	 * contiguous plane instead of two separate planes. Sws_scale needs
	 * distinct Y and UV pointers, so split the buffer manually.
	 */
	bool packed_yuv = false;
	if (n_planes == 2 && dst_data[0] && !dst_data[1]) {
		packed_yuv = true;
		dst_data[1] = dst_data[0] + srcbuf->mmap_stride[0] * self->height;
		dst_stride[1] = srcbuf->mmap_stride[0];
	}

	int rc = sws_scale(self->sws_ctx, src_data, src_stride_arr, 0,
			self->height, dst_data, dst_stride);
	if (rc < 0) {
		nvnc_log(NVNC_LOG_ERROR,
				"sws_scale failed while converting to NV12");
		/* Fall through and queue anyway; the buffer will contain
		 * whatever the driver had in it previously, but dropping would
		 * be worse.
		 */
	}

	/* The MMAP path copies frame contents into driver-allocated memory,
	 * so the source frame is no longer needed once sws_scale is done.
	 * Release it immediately to avoid holding a wayvnc buffer reference
	 */
	nvnc_frame_unmap(fb);
	nvnc_frame_unref(fb);
	srcbuf->fb = NULL;

	if (packed_yuv) {
		/* qcom-iris returns NV12/NV21 as one contiguous MMAP chunk but
		 * expects two V4L2 plane descriptors for multiplanar queueing.
		 * Both planes share the same device offset; use data_offset to
		 * locate the UV payload.
		 */
		uint32_t y_size = get_plane_size(self->src_pixfmt,
				srcbuf->mmap_stride[0], self->height, 0);
		/* get_plane_size() already halves the height for plane 1, so pass
		 * the full encoder height here. The packed buffer uses the same
		 * stride for both Y and UV.
		 */
		uint32_t uv_size = get_plane_size(self->src_pixfmt,
				srcbuf->mmap_stride[0], self->height, 1);
		off_t mem_offset = srcbuf->buffer.m.planes[0].m.mem_offset;

		srcbuf->buffer.m.planes[0].m.fd = -1;
		srcbuf->buffer.m.planes[0].bytesused = y_size;
		srcbuf->buffer.m.planes[0].length = srcbuf->planes[0].length;
		srcbuf->buffer.m.planes[0].data_offset = 0;

		srcbuf->buffer.m.planes[1].m.fd = -1;
		srcbuf->buffer.m.planes[1].m.mem_offset = mem_offset;
		srcbuf->buffer.m.planes[1].bytesused = uv_size;
		srcbuf->buffer.m.planes[1].length = srcbuf->planes[0].length;
		srcbuf->buffer.m.planes[1].data_offset = y_size;

		srcbuf->buffer.length = 2;
	} else {
		for (int i = 0; i < n_planes; ++i) {
			uint32_t plane_height = (i == 0) ? self->height
					: (self->height / 2);
			uint32_t size = get_plane_size(self->src_pixfmt,
					srcbuf->mmap_stride[i], plane_height, i);

			srcbuf->buffer.m.planes[i].m.fd = -1;
			srcbuf->buffer.m.planes[i].bytesused = size;
			srcbuf->buffer.m.planes[i].length = srcbuf->planes[i].length;
			srcbuf->buffer.m.planes[i].data_offset = 0;
		}

		srcbuf->buffer.length = n_planes;
	}
}

static void encode_buffer(struct h264_encoder_v4l2m2m* self,
		struct nvnc_frame* fb)
{
	struct h264_encoder_v4l2m2m_src_buf* srcbuf = take_src_buffer(self);
	if (!srcbuf) {
		nvnc_log(NVNC_LOG_ERROR, "Out of source buffers. Dropping frame...");
		return;
	}

	assert(!srcbuf->fb);

	nvnc_frame_ref(fb);

	/* For some reason the v4l2m2m h264 encoder in the Rapberry Pi 4 gets
	 * really glitchy unless the buffer is mapped first.
	 * This should probably be handled by the driver, but it's not.
	 */
	if (nvnc_frame_map(fb) < 0) {
		nvnc_log(NVNC_LOG_ERROR,
				"Failed to map source frame for v4l2m2m encoding");
		nvnc_frame_unref(fb);
		srcbuf->is_taken = false;
		return;
	}

	srcbuf->fb = fb;

	if (self->src_memory == V4L2M2M_SRC_MEMORY_DMABUF)
		encode_buffer_dmabuf(self, srcbuf, fb);
	else
		encode_buffer_mmap(self, srcbuf, fb);
	srcbuf->buffer.timestamp.tv_sec = fb->pts / UINT64_C(1000000);
	srcbuf->buffer.timestamp.tv_usec = fb->pts % UINT64_C(1000000);

	if (self->base.next_frame_should_be_keyframe)
		force_key_frame(self);
	self->base.next_frame_should_be_keyframe = false;
	int rc = v4l2_qbuf(self->fd, &srcbuf->buffer);
	if (rc < 0) {
		nvnc_log(NVNC_LOG_PANIC, "Failed to enqueue buffer: %m");
	}
}

static void process_fd_events(struct aml_handler* handler)
{
	struct h264_encoder_v4l2m2m* self = aml_get_userdata(handler);
	process_dst_bufs(self);
}

static int v4l2m2m_set_ctrl(struct h264_encoder_v4l2m2m* self, uint32_t id,
		int32_t value)
{
	struct v4l2_control ctrl = {
		.id = id,
		.value = value,
	};

	int rc = ioctl(self->fd, VIDIOC_S_CTRL, &ctrl);
	if (rc < 0) {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: S_CTRL id=0x%08x value=%d failed: %s",
				id, value, strerror(errno));
	} else {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: S_CTRL id=0x%08x value=%d OK", id, value);
	}

	return rc;
}

static void h264_encoder_v4l2m2m_configure(struct h264_encoder_v4l2m2m* self)
{
	int qp = self->quality;
	if (qp < 1)
		qp = 1;
	if (qp > 51)
		qp = 51;

	nvnc_log(NVNC_LOG_DEBUG,
			"v4l2m2m: configuring encoder %dx%d qp=%d",
			self->width, self->height, qp);

	/* The qcom-iris driver ignores CONSTANT_QUALITY, so also set the
	 * generic bitrate controls and the H264-specific QP controls. */
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE);
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, INT_MAX);

	/* Use a high CBR bitrate to keep the encoder from falling back to the
	 * low-quality default VBR path. */
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_BITRATE, 50000000);
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 1);

	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 1);
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_H264_MAX_QP, qp);
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, qp);
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP, qp);
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP, qp);

	/* Fallback for drivers that do support the constant-quality control. */
	v4l2m2m_set_ctrl(self, V4L2_CID_MPEG_VIDEO_CONSTANT_QUALITY, qp);
}

static bool try_set_dst_format(int fd, uint32_t dst_type, uint32_t width,
		uint32_t height)
{
	struct v4l2_format fmt = {
		.type = dst_type,
	};
	if (ioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: VIDIOC_G_FMT failed for type %u: %s",
				dst_type, strerror(errno));
		return false;
	}

	struct v4l2_pix_format_mplane* pix_fmt = &fmt.fmt.pix_mp;
	pix_fmt->pixelformat = V4L2_PIX_FMT_H264;
	pix_fmt->width = width;
	pix_fmt->height = height;

	if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: VIDIOC_S_FMT H264 %dx%d failed for type %u: %s",
				width, height, dst_type, strerror(errno));
		return false;
	}

	return true;
}

static bool detect_queue_types(int fd, uint32_t* src_type, uint32_t* dst_type)
{
	/* Some encoders (e.g. qcom-iris) do not enumerate H.264 on the capture
	 * queue until the output format is configured.  Probe by actually trying
	 * to set each queue to H.264.
	 */

	/* Try capture as destination first (normal encoder case). */
	if (try_set_dst_format(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 1920,
			1080)) {
		*src_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		*dst_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: capture queue accepts H.264 (encoder)");
		return true;
	}

	/* If capture doesn't take H.264, maybe output does (decoder or odd
	 * driver). We only want encoders, so skip this case.
	 */
	if (try_set_dst_format(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, 1920,
			1080)) {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: output queue accepts H.264 (decoder), skipping");
	}

	return false;
}

static bool is_device_capable(int fd, uint32_t width, uint32_t height)
{
	struct v4l2_capability cap = { 0 };
	int rc = ioctl(fd, VIDIOC_QUERYCAP, &cap);
	if (rc < 0) {
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: QUERYCAP failed: %s",
				strerror(errno));
		return false;
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: %s missing streaming cap",
				cap.driver);
		return false;
	}

	uint32_t src_type, dst_type;
	if (!detect_queue_types(fd, &src_type, &dst_type)) {
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: %s is not a usable H.264 encoder",
				cap.driver);
		return false;
	}

	if (!try_set_dst_format(fd, dst_type, width, height)) {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: %s failed to set destination format %dx%d",
				cap.driver, width, height);
		return false;
	}

	return true;
}

static int find_capable_device(uint32_t width, uint32_t height,
		uint32_t* src_type, uint32_t* dst_type)
{
	int fd = -1;
	DIR* dir = opendir("/dev");
	assert(dir);

	for (;;) {
		struct dirent* entry = readdir(dir);
		if (!entry)
			break;

		if (strncmp(entry->d_name, "video", 5) != 0)
			continue;

		char path[256];
		snprintf(path, sizeof(path), "/dev/%s", entry->d_name);
		fd = open(path, O_RDWR | O_CLOEXEC);
		if (fd < 0) {
			nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: cannot open %s: %s", path,
					strerror(errno));
			continue;
		}

		if (is_device_capable(fd, width, height)) {
			detect_queue_types(fd, src_type, dst_type);
			nvnc_log(NVNC_LOG_DEBUG, "Using v4l2m2m device: %s", path);
			break;
		}
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: %s is not capable, closing", path);
		close(fd);
		fd = -1;
	}

	closedir(dir);
	if (fd < 0)
		nvnc_log(NVNC_LOG_DEBUG, "No usable v4l2m2m H.264 encoder found");
	return fd;
}

static int h264_encoder_v4l2m2m_try_setup(uint32_t width, uint32_t height,
		uint32_t format, char* driver, size_t driver_size,
		uint32_t* src_type, uint32_t* dst_type, uint32_t* src_pixfmt,
		enum v4l2m2m_src_memory* src_memory,
		struct SwsContext** sws_ctx_out)
{
	int fd = find_capable_device(width, height, src_type, dst_type);
	if (fd < 0)
		return -1;

	struct h264_encoder_v4l2m2m self = {
		.fd = fd,
		.width = width,
		.height = height,
		.format = format,
		.src_type = *src_type,
		.dst_type = *dst_type,
	};

	struct v4l2_capability cap = { 0 };
	ioctl(self.fd, VIDIOC_QUERYCAP, &cap);
	strncpy(driver, (const char*)cap.driver, driver_size - 1);
	driver[driver_size - 1] = '\0';

	nvnc_log(NVNC_LOG_DEBUG,
			"v4l2m2m: probing %s %dx%d format %.4s",
			driver, width, height, (const char*)&format);

	if (set_src_fmt(&self) < 0) {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: set_src_fmt failed for %s", driver);
		goto failure;
	}

	if (set_dst_fmt(&self) < 0) {
		nvnc_log(NVNC_LOG_DEBUG,
				"v4l2m2m: set_dst_fmt failed for %s", driver);
		goto failure;
	}

	nvnc_log(NVNC_LOG_DEBUG,
			"v4l2m2m: %s setup OK src_pixfmt %.4s src_memory %d",
			driver, (const char*)&self.src_pixfmt, self.src_memory);

	if (src_pixfmt)
		*src_pixfmt = self.src_pixfmt;

	if (src_memory)
		*src_memory = self.src_memory;

	if (sws_ctx_out)
		*sws_ctx_out = self.sws_ctx;
	else if (self.sws_ctx)
		sws_freeContext(self.sws_ctx);

	return fd;

failure:
	if (self.sws_ctx)
		sws_freeContext(self.sws_ctx);
	close(fd);
	return -1;
}

bool h264_encoder_v4l2m2m_probe(uint32_t width, uint32_t height,
		uint32_t format)
{
	uint32_t src_type, dst_type, src_pixfmt;
	char driver[16];
	struct SwsContext* sws_ctx = NULL;

	nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: probing H.264 encoder %dx%d",
			width, height);

	int fd = h264_encoder_v4l2m2m_try_setup(width, height, format, driver,
			sizeof(driver), &src_type, &dst_type, &src_pixfmt, NULL,
			&sws_ctx);
	if (fd < 0) {
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: H.264 probe failed");
		return false;
	}

	nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: H.264 probe OK on %s", driver);

	close(fd);
	if (sws_ctx)
		sws_freeContext(sws_ctx);

	return true;
}

static struct h264_encoder* h264_encoder_v4l2m2m_create(uint32_t width,
		uint32_t height, uint32_t format, int quality)
{
	struct h264_encoder_v4l2m2m* self = calloc(1, sizeof(*self));
	if (!self)
		return NULL;

	self->base.impl = &h264_encoder_v4l2m2m_impl;
	self->fd = -1;
	self->width = width;
	self->height = height;
	self->format = format;
	self->quality = quality;

	self->fd = h264_encoder_v4l2m2m_try_setup(width, height, format,
			self->driver, sizeof(self->driver), &self->src_type,
			&self->dst_type, &self->src_pixfmt, &self->src_memory,
			&self->sws_ctx);
	if (self->fd < 0) {
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: create try_setup failed");
		goto failure;
	}

	nvnc_log(NVNC_LOG_DEBUG,
			"v4l2m2m: creating encoder on %s %dx%d src_pixfmt %.4s",
			self->driver, width, height, (const char*)&self->src_pixfmt);

	h264_encoder_v4l2m2m_configure(self);
	if (alloc_dst_buffers(self) < 0) {
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: alloc_dst_buffers failed");
		goto failure;
	}
	if (alloc_src_buffers(self) < 0) {
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: alloc_src_buffers failed");
		goto failure;
	}
	enqueue_dst_buffers(self);
	if (stream_on(self) < 0) {
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: stream_on failed");
		goto failure;
	}

	int flags = fcntl(self->fd, F_GETFL);
	fcntl(self->fd, F_SETFL, flags | O_NONBLOCK);

	self->handler = aml_handler_new(self->fd, process_fd_events, self, NULL);
	aml_set_event_mask(self->handler, AML_EVENT_READ);
	if (aml_start(aml_get_default(), self->handler) < 0) {
		aml_unref(self->handler);
		nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: aml_start failed");
		goto failure;
	}
	return &self->base;

failure:
	if (self->fd >= 0)
		close(self->fd);
	if (self->sws_ctx)
		sws_freeContext(self->sws_ctx);
	free(self);
	return NULL;
}

static void claim_all_src_bufs(struct h264_encoder_v4l2m2m* self)
{
	for (;;) {
		process_src_bufs(self);
		if (!any_src_buf_is_taken(self))
			break;
		usleep(10000);
	}
}

static void h264_encoder_v4l2m2m_destroy(struct h264_encoder* base)
{
	struct h264_encoder_v4l2m2m* self = (struct h264_encoder_v4l2m2m*)base;

	nvnc_log(NVNC_LOG_DEBUG, "v4l2m2m: destroying encoder %s", self->driver);

	claim_all_src_bufs(self);
	aml_stop(aml_get_default(), self->handler);
	aml_unref(self->handler);
	stream_off(self);
	free_src_buffers(self);
	free_dst_buffers(self);
	release_v4l2_buffers(self);
	if (self->fd >= 0)
		close(self->fd);
	if (self->sws_ctx)
		sws_freeContext(self->sws_ctx);
	free(self);
}

static void h264_encoder_v4l2m2m_feed(struct h264_encoder* base,
		struct nvnc_frame* fb)
{
	struct h264_encoder_v4l2m2m* self = (struct h264_encoder_v4l2m2m*)base;
	process_src_bufs(self);
	encode_buffer(self, fb);
}

struct h264_encoder_impl h264_encoder_v4l2m2m_impl = {
	.create = h264_encoder_v4l2m2m_create,
	.destroy = h264_encoder_v4l2m2m_destroy,
	.feed = h264_encoder_v4l2m2m_feed,
};
