ifdef modules
MOD_DIR := $(shell find . -name ${modules} -type d)
else
MOD_DIR := $(shell find . -maxdepth 1 -type d)
endif

MOD_DIR := $(filter-out ./.%,${MOD_DIR})

ifdef KERNELDIR
export KERNELDIR
endif

if def EXCLUDE_DIR
export EXCLUDE_DIR
endif

${MOD_DIR} : ${MOD_DIR}
	@${MAKE} -C $@
