DRI_DIR := $(shell find . -maxdepth 1 -type d)
DRI_DIR := $(basename $(patsubst ./%,%,${DRI_DIR}))
DRI_DIR := $(filter-out ${EXCLUDE_DIRS},${DRI_DIR})

${DRI_DIR} : ${DRI_DIR}
	${MAKE} -C $@
