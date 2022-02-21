#
#Executable name
#
EXE_NAME=x

#
#Folder to place compiled objects
#
BUILDDIR = build

#
#Verbose mode
#
VERBOSE=no

#
#Colorize ouput
#
COLORIZE=yes

#
#Enable binary creation
#
MAKE_BINARY=no

#
#Enable executable creation
#
MAKE_EXECUTABLE=yes

#
#Enable shared library creation
#
MAKE_SHARED_LIB=no

#
#Enable static library creation
#
MAKE_STATIC_LIB=no

#
#Enable MAP-file creation
#
CREATE_MAP=no

#
#Tool-chain prefix
#
TCHAIN =

#
#CPU specific options
#
MCPU +=
#
#C language dialect
#
CDIALECT = gnu99

#
#C++ language dialect
#
# CXXDIALECT =

#
#Optimization
#
OPT_LVL = 2

#
#Debug information options
#
DBG_OPTS = -gdwarf-2 -ggdb -g

#
#Additional C flags
#
CFLAGS   += -fdata-sections -ffunction-sections 
CFLAGS   += -fsingle-precision-constant

CFLAGS   += -Wno-pointer-sign
CFLAGS   += -fmessage-length=0
CFLAGS   += -fno-exceptions
CFLAGS   += -fno-move-loop-invariants -ffreestanding

CFLAGS   += -Wall -Wextra -Wstrict-prototypes  -Wdisabled-optimization -Wformat=2 -Winit-self -Wlogical-op -Wmissing-include-dirs
CFLAGS   += -Wshadow -Wsign-conversion -Wstrict-overflow=5 -Wswitch-default -Wno-unused
CFLAGS   += -pedantic -Wcast-qual -Wcast-align

# CFLAGS   += -Werror
# CFLAGS   += -Wundef 
# CFLAGS   += -Wmissing-declarations 
# CFLAGS   += -Wredundant-decls

#
#Additional CPP flags
#
CXXFLAGS += -fdata-sections -ffunction-sections 
CXXFLAGS += -fsingle-precision-constant

#
#Additional linker flags
#
# LDFLAGS  +=

#
#Additional link objects
#
#EXT_OBJECTS +=

#
#Additional static libraries
#
# EXT_LIBS +=c

#
#Preprocessor definitions
#
PPDEFS  +=


#
#Include directories
#
INCDIR  += ..

#
#Source files
#
SOURCES += $(wildcard *.cpp)
SOURCES += $(wildcard ../serial_port/*.cpp)

#
#Linker scripts
#
# LDSCRIPT += ldscript/app.ld

include core.mk

.PHONY : tags
tags:
	ctags -R $(SOURCES) $(INCDIR) 
