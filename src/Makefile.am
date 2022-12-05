# Note: plugindir is set in configure

plugin_LTLIBRARIES = libgstcedar_h264enc.la

# sources used to compile this plug-in
libgstcedar_h264enc_la_SOURCES = gstcedarh264enc.c gstcedarh264enc.h \
	h264enc.c h264enc.h

# compiler and linker flags used to compile this plugin, set in configure.ac
libgstcedar_h264enc_la_CFLAGS = $(GST_CFLAGS)
libgstcedar_h264enc_la_LIBADD = $(GST_LIBS)
libgstcedar_h264enc_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS) -lpthread -lm -lcdc_base -lVE  -lvencoder -lMemAdapter
libgstcedar_h264enc_la_LIBTOOLFLAGS = --tag=disable-static

# headers we need but don't want installed
noinst_HEADERS = gstcedarh264enc.h h264enc.h
