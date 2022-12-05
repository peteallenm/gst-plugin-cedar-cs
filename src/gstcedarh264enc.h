/*
 * Cedar H264 Encoder Plugin
 * Copyright (C) 2014 Enrico Butera <ebutera@users.sourceforge.net>
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

#ifndef __GST_CEDAR_H264ENC_H__
#define __GST_CEDAR_H264ENC_H__

#include <gst/gst.h>
#include <gst/video/video.h>

#include "h264enc.h"

G_BEGIN_DECLS

/* #defines don't like whitespacey bits */
#define GST_TYPE_CEDAR_H264ENC			(gst_cedarh264enc_get_type())
#define GST_CEDAR_H264ENC(obj)			(G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_CEDAR_H264ENC, GstCedarH264Enc))
#define GST_CEDAR_H264ENC_CLASS(klass)		(G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_CEDAR_H264ENC, GstCedarH264EncClass))
#define GST_IS_CEDAR_H264ENC(obj)		(G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_CEDAR_H264ENC))
#define GST_IS_CEDAR_H264ENC_CLASS(klass)	(G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_CEDAR_H264ENC))

typedef struct _GstCedarH264Enc		GstCedarH264Enc;
typedef struct _GstCedarH264EncClass	GstCedarH264EncClass;

struct _GstCedarH264Enc
{
	GstElement element;

	GstPad *sinkpad, *srcpad;

    gint32 bitrate;
	gint32 pic_init_qp;
	gint32 keyframe_interval;
	gboolean always_copy;
  
	int width;
	int height;

	h264enc *enc;
};

struct _GstCedarH264EncClass 
{
	GstElementClass parent_class;
};

GType gst_cedarh264enc_get_type(void);

G_END_DECLS

#endif /* __GST_CEDAR_H264ENC_H__ */
