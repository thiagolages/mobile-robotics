IMAGE ?= docker.io/thiagolages/template-robotics:latest
NAME=robotics

build:
	docker build -t $(IMAGE) -f docker/Dockerfile --build-arg USERNAME=$(shell id -un) .

run-headless: build
	docker compose up -d --remove-orphans

run-detached:
	USERNAME=$(shell id -un) docker compose up -d --remove-orphans

run:
	$(MAKE) run-detached
	docker exec -it $(NAME) bash

restart:
	$(MAKE) stop
	$(MAKE) run

shell:
	docker exec -it $(NAME) bash

stop:
	docker kill $(NAME) || true

clean:
	docker compose down -v || true
	docker image rm $(IMAGE) || true
