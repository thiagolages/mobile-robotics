IMAGE ?= docker.io/thiagolages/template-robotics:latest
NAME=robotics

build:
	docker build -t $(IMAGE) -f docker/Dockerfile .

run-headless: build
	docker compose up -d --remove-orphans

run-detached:
	docker compose up -d --remove-orphans

run:
	$(MAKE) run-detached
	docker exec -it $(NAME) bash

shell:
	docker exec -it $(NAME) bash

stop:
	docker kill $(NAME) || true

clean:
	docker compose down -v || true
	docker image rm $(IMAGE) || true
