#######################################
# build tool's names
#######################################
CC=$(TCHAIN)gcc
CXX=$(TCHAIN)g++
OC=$(TCHAIN)objcopy
AR=$(TCHAIN)ar
RL=$(TCHAIN)ranlib
SZ=$(TCHAIN)size

#######################################
# color @ verbose settings
#######################################
ifeq ($(strip $(COLORIZE)),yes)
	CLRED=\e[31m
	CLGRN=\e[32m
	CLYEL=\e[33m
	CLRST=\e[0m
	VECHO_=printf
else
	CLRED=
	CLGRN=
	CLYEL=
	CLRST=
	VECHO_=printf
endif

ifneq ($(VERBOSE),yes)
	Q:=@
	VECHO=@$(VECHO_)
else
	Q:= 
	VECHO=@true
endif

#######################################
# manage output files suffixes
#######################################
ifeq ($(SHARED_LIB_EXT),)
	SHARED_LIB_EXT=.so
else
	SHARED_LIB_EXT:=.$(SHARED_LIB_EXT)
endif

ifeq ($(STATIC_LIB_EXT),)
	STATIC_LIB_EXT=.a
else
	STATIC_LIB_EXT:=.$(STATIC_LIB_EXT)
endif

ifneq ($(EXE_EXT),)
	EXE_EXT:=.$(EXE_EXT)
endif

#######################################
# C/C++ flags tuning
#######################################
FLAGS=-c -Wall -O$(OPT_LVL)
FLAGS+=$(DBG_OPTS)
FLAGS+=-MMD -MP 
FLAGS+=$(MCPU) $(addprefix -I,$(INCDIR)) $(addprefix -D,$(PPDEFS)) 

ifneq ($(CDIALECT),)
CFLAGS+=-std=$(CDIALECT)
endif

ifneq ($(CXXDIALECT),)
CXXFLAGS+=-std=$(CXXDIALECT)
endif

LDFLAGS+=$(MCPU) 
ifeq ($(strip $(CREATE_MAP)),yes)
LDFLAGS+=-Wl,-Map=$(BUILDDIR)/$(EXE_NAME).map
endif
LDFLAGS+=$(addprefix -T,$(LDSCRIPT))

#######################################
# search path
#######################################
vpath %.c $(SRC_PATH)
vpath %.s $(SRC_PATH)
vpath %.S $(SRC_PATH)
vpath %.cpp $(SRC_PATH)
vpath %.cxx $(SRC_PATH)
vpath %.c++ $(SRC_PATH)
vpath %.cc $(SRC_PATH)
vpath %.C $(SRC_PATH)
VPATH = $(SRC_PATH)

#######################################
# sort sources & generate object names
#######################################
CXX_SOURCES:=$(filter %.cpp %.cxx %.c++ %.cc %.C, $(SOURCES))
CXX_OBJECTS:=$(CXX_SOURCES:.cpp=.o)
CXX_OBJECTS:=$(CXX_OBJECTS:.cxx=.o)
CXX_OBJECTS:=$(CXX_OBJECTS:.c++=.o)
CXX_OBJECTS:=$(CXX_OBJECTS:.cc=.o)
CXX_OBJECTS:=$(CXX_OBJECTS:.C=.o)

C_SOURCES:=$(filter %.c, $(SOURCES))
C_OBJECTS:=$(C_SOURCES:.c=.o)

S_SOURCES:=$(filter %.s %.S, $(SOURCES))
S_OBJECTS:=$(S_SOURCES:.s=.o)
S_OBJECTS:=$(S_OBJECTS:.S=.o)

CXX_OBJECTS:=$(addprefix $(BUILDDIR)/, $(CXX_OBJECTS))
C_OBJECTS:=$(addprefix $(BUILDDIR)/, $(C_OBJECTS))
S_OBJECTS:=$(addprefix $(BUILDDIR)/, $(S_OBJECTS))

#######################################
# decide which linker to use C or C++
#######################################
ifeq ($(strip $(CXX_SOURCES)),)
	LD=$(CC)
else
	LD=$(CXX)
endif

#######################################
# final list of objects
#######################################
LINK_OBJECTS=$(S_OBJECTS) $(C_OBJECTS) $(CXX_OBJECTS) $(EXT_OBJECTS)

#######################################
# make depend on makefile & core.mk
#######################################
SELFDEP=Makefile core.mk

#######################################
# list of desired artefacts
#######################################
EXECUTABLE:=$(BUILDDIR)/$(EXE_NAME)$(EXE_EXT)
BINARY:=$(BUILDDIR)/$(basename $(EXE_NAME)).bin
SHARED_LIB:=$(BUILDDIR)/$(basename lib$(EXE_NAME))$(SHARED_LIB_EXT)
STATIC_LIB:=$(BUILDDIR)/$(basename lib$(EXE_NAME))$(STATIC_LIB_EXT)

ifeq ($(strip $(MAKE_BINARY)),yes)
	ARTEFACTS+=$(BINARY)
endif
ifeq ($(strip $(MAKE_EXECUTABLE)),yes)
	ARTEFACTS+=$(EXECUTABLE)
endif

ifeq ($(strip $(MAKE_SHARED_LIB)),yes)
	ARTEFACTS+=$(SHARED_LIB)
	CFLAGS+=-fpic
endif

ifeq ($(strip $(MAKE_STATIC_LIB)),yes)
	ARTEFACTS+=$(STATIC_LIB)
endif

#######################################
# build & clear targets
#######################################
.PHONY : all clean debug
all: $(ARTEFACTS)
debug:
	@echo "C:"
	@echo $(C_SOURCES)
	@echo "CXX:"
	@echo $(CXX_SOURCES)
	@echo "S:"
	@echo $(S_SOURCES)
	@echo "Co:"
	@echo $(C_OBJECTS)
	@echo "CXXo:"
	@echo $(CXX_OBJECTS)
	@echo "So"
	@echo $(S_OBJECTS)

$(SHARED_LIB): $(LINK_OBJECTS)
	$(VECHO)  ' [$(CLRED)L$(CLRST)]   $(CLRED)$@$(CLRST) ...\n'
	$(Q)$(LD) -shared -o $@  $(LDFLAGS) $(LINK_OBJECTS) $(addprefix -L,$(LIBDIR)) $(addprefix -l,$(EXT_LIBS)) 

$(STATIC_LIB): $(LINK_OBJECTS)
	$(VECHO)  ' [$(CLRED)AR$(CLRST)]  $(CLRED)$@$(CLRST) ...\n'
	$(Q)$(AR) -rc $@  $(LINK_OBJECTS) 
	$(VECHO)  ' [$(CLRED)RL$(CLRST)]  $(CLRED)$@$(CLRST) ...\n'
	$(Q)$(RL) $@ 

$(EXECUTABLE): $(LINK_OBJECTS)
	$(VECHO)  ' [$(CLRED)L$(CLRST)]   $(CLRED)$@$(CLRST) ...\n'
	$(Q)$(LD) -o $@  $(LDFLAGS) $(LINK_OBJECTS) $(addprefix -L,$(LIBDIR)) $(addprefix -l,$(EXT_LIBS)) 
	$(Q)$(SZ) $@

$(BINARY) : $(EXECUTABLE)
	$(VECHO) ' [$(CLRED)B$(CLRST)]   $(CLRED)$@$(CLRST) ...\n'
	$(Q)$(OC) -O binary $< $(@)

#######################################
# assebler targets
#######################################
$(BUILDDIR)/%.o: %.s $(SELFDEP)
	@mkdir -p $(@D)
	$(VECHO) ' [$(CLGRN)S$(CLRST)]   $< ...\n'
	$(Q)$(CC) $(FLAGS) $(CFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.S $(SELFDEP)
	@mkdir -p $(@D)
	$(VECHO) ' [$(CLGRN)S$(CLRST)]   $< ...\n'
	$(Q)$(CC) $(FLAGS) $(CFLAGS) $< -o $@

#######################################
# C targets
#######################################
$(BUILDDIR)/%.o: %.c $(SELFDEP)
	@mkdir -p $(@D)
	$(VECHO) ' [$(CLGRN)C$(CLRST)]   $< ...\n'
	$(Q)$(CC) $(FLAGS) $(CFLAGS) $< -o $@

#######################################
# C++ targets
#######################################
$(BUILDDIR)/%.o: %.cpp $(SELFDEP)
	@mkdir -p $(@D)
	$(VECHO) ' [$(CLYEL)C++$(CLRST)] $< ...\n'
	$(Q)$(CXX) $(FLAGS) $(CXXFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.C $(SELFDEP)
	@mkdir -p $(@D)
	$(VECHO) ' [$(CLYEL)C++$(CLRST)] $< ...\n'
	$(Q)$(CXX) $(FLAGS) $(CXXFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.cxx $(SELFDEP)
	@mkdir -p $(@D)
	$(VECHO) ' [$(CLYEL)C++$(CLRST)] $< ...\n'
	$(Q)$(CXX) $(FLAGS) $(CXXFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.cc $(SELFDEP)
	@mkdir -p $(@D)
	$(VECHO) ' [$(CLYEL)C++$(CLRST)] $< ...\n'
	$(Q)$(CXX) $(FLAGS) $(CXXFLAGS) $< -o $@

$(BUILDDIR)/%.o: %.c++ $(SELFDEP)
	@mkdir -p $(@D)
	$(VECHO) ' [$(CLYEL)C++$(CLRST)] $< ...\n'
	$(Q)$(CXX) $(FLAGS) $(CXXFLAGS) $< -o $@

clean:
	$(Q)rm -fr $(BUILDDIR)

#######################################
# include header dependencies
#######################################
-include $(C_OBJECTS:.o=.d)
-include $(CXX_OBJECTS:.o=.d)
