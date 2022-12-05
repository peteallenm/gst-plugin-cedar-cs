/*
 * Cedar H264 Encoder Plugin
 * Copyright (C) 2014 Enrico Butera <ebutera@users.sourceforge.net>
 *
 * Byte stream utils:
 * Copyright (c) 2014 Jens Kuske <jenskuske@gmail.com>
 *
 * Gst template code:
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * SECTION:element-cedar_h264enc
 *
 * H264 Encoder plugin using CedarX hardware engine
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -ve videotestsrc ! cedar_h264enc ! h264parse ! matroskamux ! filesink location="cedar.mkv"
 * ]|
 * </refsect2>
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <gst/gst.h>

#include "gstcedarh264enc.h"

GST_DEBUG_CATEGORY_STATIC(gst_cedarh264enc_debug);
#define GST_CAT_DEFAULT gst_cedarh264enc_debug

#define BITRATE_MULT 1024

enum
{
	PROP_0,
    PROP_BITRATE,
	PROP_QP,
	PROP_KEYFRAME_INTERVAL,
	PROP_ALWAYS_COPY
};

/* the capabilities of the inputs and outputs.
 *
 * describe the real formats here.
 */
static GstStaticPadTemplate sink_factory = GST_STATIC_PAD_TEMPLATE("sink",
	GST_PAD_SINK,
	GST_PAD_ALWAYS,
	GST_STATIC_CAPS (
		"video/x-raw, "
		"format = (string) NV12, "
		"width = (int) [16,1280], "
		"height = (int) [16,720]"
		/*"framerate=(fraction)[1/1,25/1]"*/
	)
);

static GstStaticPadTemplate src_factory = GST_STATIC_PAD_TEMPLATE("src",
	GST_PAD_SRC,
	GST_PAD_ALWAYS,
	GST_STATIC_CAPS (
		"video/x-h264, "
		"stream-format = (string) byte-stream, "
		"alignment = (string) nal, "
		"profile = (string) { main }"
	)
);

G_DEFINE_TYPE(GstCedarH264Enc, gst_cedarh264enc, GST_TYPE_ELEMENT);

static void gst_cedarh264enc_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec);
static void gst_cedarh264enc_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec);

static gboolean gst_cedarh264enc_set_caps(GstPad *pad, GstCaps *caps);
static GstFlowReturn gst_cedarh264enc_chain(GstPad *pad, GstObject *parent, GstBuffer *buf);

static GstStateChangeReturn gst_cedarh264enc_change_state(GstElement *element, GstStateChange transition);

/* initialize the cedar_h264enc's class */
static void gst_cedarh264enc_class_init(GstCedarH264EncClass *klass)
{
	GObjectClass *gobject_class;
	GstElementClass *gstelement_class;

	gobject_class = (GObjectClass *) klass;
	gstelement_class = (GstElementClass *) klass;
	gst_element_class_set_details_simple(gstelement_class,
		"cedar_h264enc",
		"CedarX H264 Encoder",
		"H264 Encoder Plugin for CedarX hardware",
		"Enrico Butera <ebutera@users.berlios.de>, Kyle Hu <kyle.hu.gz@gmail.com>, George Talusan <george.talusan@gmail.com>");

	gst_element_class_add_pad_template(gstelement_class, gst_static_pad_template_get(&src_factory));
	gst_element_class_add_pad_template(gstelement_class, gst_static_pad_template_get(&sink_factory));
  
	gobject_class->set_property = gst_cedarh264enc_set_property;
	gobject_class->get_property = gst_cedarh264enc_get_property;

	gstelement_class->change_state = gst_cedarh264enc_change_state;

    g_object_class_install_property (gobject_class, PROP_BITRATE,
		g_param_spec_int ("bitrate", "BITRATE", "H264 target bitrate (kbits)",
		1000, 20000, 5000, G_PARAM_READWRITE));
        
	g_object_class_install_property (gobject_class, PROP_QP,
		g_param_spec_int ("qp", "QP", "H264 quantization parameters",
		0, 47, 15, G_PARAM_READWRITE));

	g_object_class_install_property (gobject_class, PROP_KEYFRAME_INTERVAL,
		g_param_spec_int ("keyint", "keyframe-interval", "Keyframe Interval",
		0, 500, 0, G_PARAM_READWRITE));

	g_object_class_install_property (gobject_class, PROP_ALWAYS_COPY,
		g_param_spec_boolean ("always-copy", "Always Copy", "Always Copy Buffers",
		TRUE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
}

static gboolean gst_cedarh264enc_sink_event(GstPad *pad, GstObject *parent, GstEvent *event)
{
	gboolean ret;
	GstCaps *caps;
	switch (GST_EVENT_TYPE(event)) {
	case GST_EVENT_CAPS:
		gst_event_parse_caps(event, &caps);
		ret = gst_cedarh264enc_set_caps(pad, caps);
		break;

	default:
		ret = gst_pad_event_default(pad, parent, event);
		break;
	}
	return ret;
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
static void gst_cedarh264enc_init(GstCedarH264Enc *filter)
{
	filter->sinkpad = gst_pad_new_from_static_template(&sink_factory, "sink");
	gst_pad_set_event_function(filter->sinkpad, GST_DEBUG_FUNCPTR(gst_cedarh264enc_sink_event));
	gst_pad_set_chain_function(filter->sinkpad, GST_DEBUG_FUNCPTR(gst_cedarh264enc_chain));

	filter->srcpad = gst_pad_new_from_static_template(&src_factory, "src");
	gst_pad_use_fixed_caps(filter->srcpad);

	gst_element_add_pad(GST_ELEMENT(filter), filter->sinkpad);
	gst_element_add_pad(GST_ELEMENT(filter), filter->srcpad);

    
	filter->bitrate = 0; // If bitrate is 0 we will be in qp mode
	filter->pic_init_qp = 15;
	filter->keyframe_interval = 0;
	filter->always_copy = TRUE;
}

static void gst_cedarh264enc_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec)
{
	GstCedarH264Enc *filter = GST_CEDAR_H264ENC(object);

	switch (prop_id) {
    case PROP_BITRATE:
		filter->bitrate = g_value_get_int(value) * BITRATE_MULT;
		break;
	case PROP_QP:
		filter->pic_init_qp = g_value_get_int(value);
		break;

	case PROP_KEYFRAME_INTERVAL:
		filter->keyframe_interval = g_value_get_int(value);
		break;

	case PROP_ALWAYS_COPY:
		filter->always_copy = g_value_get_boolean(value);
		break;

	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		break;
	}
}

static void gst_cedarh264enc_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec)
{
	GstCedarH264Enc *filter = GST_CEDAR_H264ENC(object);

	switch (prop_id) {
    case PROP_BITRATE:
		g_value_set_int(value, filter->bitrate / BITRATE_MULT);
		break;
	case PROP_QP:
		g_value_set_int(value, filter->pic_init_qp);
		break;

	case PROP_KEYFRAME_INTERVAL:
		g_value_set_int(value, filter->keyframe_interval);
		break;

	case PROP_ALWAYS_COPY:
		g_value_set_boolean(value, filter->always_copy);
		break;

	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		break;
	}
}

/* GstElement vmethod implementations */

/* this function handles the link with other elements */
static gboolean gst_cedarh264enc_set_caps(GstPad *pad, GstCaps *caps)
{
	GstCedarH264Enc *filter;
	GstPad *otherpad;
	GstCaps *othercaps;
	GstVideoInfo vinfo;

	filter = GST_CEDAR_H264ENC(gst_pad_get_parent(pad));
	otherpad = (pad == filter->srcpad) ? filter->sinkpad : filter->srcpad;

	if (pad == filter->sinkpad) {
		int ret;
		int fps_num, fps_den;

		gst_video_info_from_caps(&vinfo, caps);
		filter->width = vinfo.width;
		filter->height = vinfo.height;
		fps_num = vinfo.fps_n;
		fps_den = vinfo.fps_d;

		othercaps = gst_caps_copy(gst_pad_get_pad_template_caps(filter->srcpad));
		gst_caps_set_simple(othercaps,
			"width", G_TYPE_INT, filter->width,
			"height", G_TYPE_INT, filter->height,
			"framerate", GST_TYPE_FRACTION, fps_num, fps_den,
			"profile", G_TYPE_STRING, "main", NULL);

		gst_object_unref(filter);
		ret = gst_pad_set_caps(otherpad, othercaps);
		gst_caps_unref(othercaps);

		return ret;
	}

	gst_object_unref(filter);
	return gst_pad_set_caps(otherpad, caps);
}

/* chain function
 * this function does the actual processing
 */
static GstFlowReturn gst_cedarh264enc_chain(GstPad *pad, GstObject *parent, GstBuffer *buf)
{
	GstCedarH264Enc *filter;
	GstBuffer *outbuf;
	GstMapInfo info;
	gsize len0, len1;
    gsize pps_sps_len = 0;
    
	filter = GST_CEDAR_H264ENC(GST_OBJECT_PARENT(pad));

	if (!gst_buffer_map(buf, &info, GST_MAP_READ)) {
		// TODO: needed?
		GST_WARNING("Received empty buffer");
		outbuf = gst_buffer_new();
		GST_BUFFER_TIMESTAMP(outbuf) = GST_BUFFER_TIMESTAMP(buf);
		return gst_pad_push(filter->srcpad, outbuf);
	}

	if (!filter->enc) {
		struct h264enc_params p = {
			.width = filter->width,
			.height = filter->height,
			.src_width = filter->width,
			.src_height = filter->height,
			.src_format = H264_FMT_NV12,
			.profile_idc = 77,	// Main Profile
			.level_idc = 41,
			.entropy_coding_mode = H264_EC_CABAC,
			.qp = filter->pic_init_qp,
            .bitrate = filter->bitrate,
            //.bitrate = 5 * 1024 * 1024,
			.keyframe_interval = filter->keyframe_interval
		};

		filter->enc = h264enc_new(&p);
        
		if (!filter->enc) {
			GST_ERROR("Cannot initialize H.264 encoder");
			return GST_FLOW_ERROR;
		}
        
        len0 = h264enc_get_initial_bytestream_length(filter->enc);
        if(len0)
        {
            outbuf = gst_buffer_new_and_alloc(len0);
            gst_buffer_fill(outbuf, 0, h264enc_get_intial_bytestream_buffer(filter->enc), len0);
            gst_pad_push(filter->srcpad, outbuf);
        }
        else
            GST_ERROR("Pete: No initial bytestream\n");
	}

    
	h264enc_set_input_buffer(filter->enc, info.data, info.size);

	if (!h264enc_encode_picture(filter->enc)) {
		return GST_FLOW_ERROR;
	}
    
    
	len0 = h264enc_get_bytestream_length(filter->enc, 0);
	len1 = h264enc_get_bytestream_length(filter->enc, 1);
    
    
    // Send SPS and PPS with keyframes
    if(h264enc_is_keyframe(filter->enc))
    {
        pps_sps_len = h264enc_get_initial_bytestream_length(filter->enc);
    }
        
	if (filter->always_copy) {
        gsize offset = 0;
        
		outbuf = gst_buffer_new_and_alloc(len0 + len1 + pps_sps_len);
        
        if(h264enc_is_keyframe(filter->enc))
        {
            gst_buffer_fill(outbuf, 0, h264enc_get_intial_bytestream_buffer(filter->enc), pps_sps_len);
            offset += pps_sps_len;
        }
        
		gst_buffer_fill(outbuf, offset, h264enc_get_bytestream_buffer(filter->enc, 0), len0);
        offset += len0;
        
        if(len1 > 0)
        {
            gst_buffer_fill(outbuf, offset, h264enc_get_bytestream_buffer(filter->enc, 1), len1);
        }
	} else {
        gsize offset = 0;
        printf("Nooooo this may not work!!!!\n");
        if(h264enc_is_keyframe(filter->enc))
        {
            outbuf = gst_buffer_new_wrapped_full(0,
                h264enc_get_intial_bytestream_buffer(filter->enc),
                len0 + len1 + pps_sps_len, 0, pps_sps_len, 0, 0);
                offset = pps_sps_len;
            
            gst_buffer_fill(outbuf, offset, h264enc_get_bytestream_buffer(filter->enc, 0), len0);
            offset += len0;
        }
        else
        {
            outbuf = gst_buffer_new_wrapped_full(0,
                h264enc_get_bytestream_buffer(filter->enc, 0),
                len0 + len1, 0, len0, 0, 0);
            offset = len0;
        }
        
        if(len1 > 0)
        {
            gst_buffer_fill(outbuf, len0, h264enc_get_bytestream_buffer(filter->enc, 1), len1);
        }
	}
	GST_BUFFER_TIMESTAMP(outbuf) = GST_BUFFER_TIMESTAMP(buf);
    
    h264enc_done_outputbuffer(filter->enc);
    
	gst_buffer_unmap(buf, &info);
	gst_buffer_unref(buf);

	return gst_pad_push(filter->srcpad, outbuf);
}

static GstStateChangeReturn gst_cedarh264enc_change_state(GstElement *element, GstStateChange transition)
{
	GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
	GstCedarH264Enc *filter = GST_CEDAR_H264ENC(element);

	switch (transition) {
	case GST_STATE_CHANGE_NULL_TO_READY:
		break;

	case GST_STATE_CHANGE_READY_TO_PAUSED:
		break;

	case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
		break;

	default:
		// silence compiler warning...
		break;
	}

	ret = GST_ELEMENT_CLASS(gst_cedarh264enc_parent_class)->change_state(element, transition);
	if (ret == GST_STATE_CHANGE_FAILURE)
		return ret;

	switch (transition) {
	case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
		break;

	case GST_STATE_CHANGE_PAUSED_TO_READY:
		h264enc_free(filter->enc);
		filter->enc = NULL;
		break;

	case GST_STATE_CHANGE_READY_TO_NULL:
		break;

	default:
		// silence compiler warning...
		break;
	}

	return ret;
}

/* entry point to initialize the plug-in */
static gboolean cedar_h264enc_init(GstPlugin *cedar_h264enc)
{
	// debug category for fltering log messages
	GST_DEBUG_CATEGORY_INIT(gst_cedarh264enc_debug, "cedar_h264enc", 0, "CedarX H264 Encoder");

	return gst_element_register(cedar_h264enc, "cedar_h264enc", GST_RANK_NONE, GST_TYPE_CEDAR_H264ENC);
}

/* PACKAGE: this is usually set by autotools depending on some _INIT macro
 * in configure.ac and then written into and defined in config.h, but we can
 * just set it ourselves here in case someone doesn't use autotools to
 * compile this code. GST_PLUGIN_DEFINE needs PACKAGE to be defined.
 */
#ifndef PACKAGE
#define PACKAGE "myfirstcedar_h264enc"
#endif

// gstreamer looks for this structure to register cedar_h264encs
GST_PLUGIN_DEFINE (
	GST_VERSION_MAJOR,
	GST_VERSION_MINOR,
	cedar_h264enc,
	"CedarX H264 Encoder",
	cedar_h264enc_init,
	VERSION,
	"LGPL",
	"Sunxi",
	"http://github.com/gtalusan/gst-plugin-cedar"
)
