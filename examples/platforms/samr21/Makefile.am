#
#  Copyright (c) 2016, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

include $(abs_top_nlbuild_autotools_dir)/automake/pre.am

lib_LIBRARIES                                                                           = libopenthread-samr21.a

libopenthread_samr21_a_CPPFLAGS                                                         = \
    -D__ATSAMR21G18A__                                                                    \
    -I$(top_srcdir)/include                                                               \
    -I$(top_srcdir)/examples/platforms                                                    \
    -I$(top_srcdir)/src/core                                                              \
    -I$(top_srcdir)/third_party/microchip/cmsis                                           \
    -I$(top_srcdir)/third_party/microchip/samr21/include                                  \
    -I$(top_srcdir)/third_party/microchip/samr21/include/instance                         \
    -I$(top_srcdir)/third_party/microchip/samr21/include/component                        \
    -I$(top_srcdir)/third_party/microchip/samr21/include/pio                              \
    -I$(top_srcdir)/third_party/microchip/samr21/source                                   \
    -Wno-unknown-pragmas                                                                  \
    -Wno-sign-compare                                                                     \
    -Wno-unused-function                                                                  \
    -Wno-unused-parameter                                                                 \
    -Wno-shadow                                                                           \
    -Wno-type-limits                                                                      \
    -Wno-unused-variable                                                                  \
    -Wno-missing-field-initializers                                                       \
    -fno-strict-aliasing                                                                  \
    -Wno-pedantic                                                                         \
    $(NULL)

libopenthread_samr21_a_SOURCES                                                          = \
    alarm.c                                                                               \
    flash.c                                                                               \
    misc.c                                                                                \
    platform.c                                                                            \
    radio.c                                                                               \
    random.c                                                                              \
    startup-gcc.c                                                                         \
    uart.c                                                                                \
    gpio.c                                                                                \
    tinymt32/tinymt32.c                                                                   \
    @top_builddir@/third_party/microchip/samr21/source/gcc/startup_samr21.c               \
    @top_builddir@/third_party/microchip/samr21/source/system_samr21.c                    \
    $(NULL)
    
if OPENTHREAD_ENABLE_DIAG
libopenthread_samr21_a_SOURCES                                                         += \
    diag.c                                                                                \
    $(NULL)
endif

noinst_HEADERS                                                                          = \
    platform-samr21.h                                                                     \
    $(NULL)

Dash                                                                                    = -
libopenthread_samr21_a_LIBADD                                                           = \
    $(shell find $(top_builddir)/examples/platforms/utils $(Dash)type f $(Dash)name "*.o")

include $(abs_top_nlbuild_autotools_dir)/automake/post.am
