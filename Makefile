# Replace old SVN definitions (SVNNAME, SVNMAXREV)
# by Git definitions

# Determine current Git version and nearest Git tag
GIT_FULL_VERSION = $(shell git describe --tags --abbrev=6)
NEAREST_TAG = $(shell git describe --tags --abbrev=0)
GIT_NEAREST_TAG = $(if $(NEAREST_TAG),$(NEAREST_TAG),0)
GITNAME="\"fifisdr3\""

export GIT_FULL_VERSION
export GIT_NEAREST_TAG
export GITNAME

$(info $$GIT_NEAREST_TAG is [${GIT_NEAREST_TAG}])

# Select the RTOS brand (FREERTOS, RTX)
export RTOS_BRAND=FREERTOS

# The only task is to call the Makefile in the source directory
all:
	@cd gcc && $(MAKE)

clean:
	@cd gcc && $(MAKE) clean
