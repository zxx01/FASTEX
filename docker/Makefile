IMAGE_NAME := ros:fastex_experiment
CATKIN_WS_DIR_ := "$(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../../..)"
HTTP_PROXY ?=
HTTPS_PROXY ?=
DOCKER ?= docker
USER_ID := $(shell id -u)
GROUP_ID := $(shell id -g)

all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
    # @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"

build:
	@$(DOCKER) build --no-cache --network=host \
	 --build-arg HTTP_PROXY=$(HTTP_PROXY) \
	 --build-arg HTTPS_PROXY=$(HTTPS_PROXY) \
	 --build-arg http_proxy=$(HTTP_PROXY) \
	 --build-arg https_proxy=$(HTTPS_PROXY) \
	 --build-arg USER_ID=$(USER_ID) \
	 --build-arg GROUP_ID=$(GROUP_ID) \
	 --build-arg CATKIN_WS_DIR=$(CATKIN_WS_DIR_) --tag $(IMAGE_NAME) -f ./cave_exp.Dockerfile .

# pull:
# 	@sudo docker pull $(IMAGE_NAME)

clean:
	@$(DOCKER) rmi -f $(IMAGE_NAME)
