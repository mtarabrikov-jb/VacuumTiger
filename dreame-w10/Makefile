# Dreame W10 "over-ava" bridge — build / deploy / capture.
#
# The robot (aarch64, glibc 2.23) runs `ava`, which owns /dev/ttyS4 (MCU) and
# /dev/ttyS3 (LDS). We inject a read-only serial tap and relay so a VacuumTiger
# driver can consume the streams while `ava` keeps the robot safe. See README.md.

# Set your robot's SSH target here or via the environment: `make ROBOT=root@<ip> ...`
# (kept out of the repo on purpose — no device addresses committed).
ROBOT      ?= root@ROBOT-IP-NOT-SET
REMOTE_DIR ?= /data/w10bridge
SSH        := ssh -o ConnectTimeout=12 $(ROBOT)
HOST        = $(word 2,$(subst @, ,$(ROBOT)))

.DEFAULT_GOAL := help

.PHONY: help
help: ## Show this help
	@echo "dreame-w10 bridge — targets:"
	@grep -hE '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| sort | awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-14s\033[0m %s\n",$$1,$$2}'
	@echo
	@echo "Flow:  make docker → make upload → make inject → make decode"
	@echo "       (inject RESTARTS ava — robot idle on the dock)"

.PHONY: check
check: ## Build host crates (proto/shm/relay/decode) + run proto tests
	cargo build
	cargo test -p dreame-w10-proto

.PHONY: docker
docker: ## Cross-compile avatap.so + avatap-relay for the robot (aarch64) via Docker
	docker build -f build/Dockerfile -t w10-bridge-build .
	@mkdir -p out
	@cid=$$(docker create w10-bridge-build) && \
		docker cp $$cid:/out/avatap.so out/avatap.so && \
		docker cp $$cid:/out/avatap-relay out/avatap-relay && \
		docker rm $$cid >/dev/null
	@echo ">> built out/avatap.so + out/avatap-relay (aarch64)"

.PHONY: upload
upload: ## Push out/* + inject.sh to $(REMOTE_DIR) on the robot (tar-over-ssh, no sftp)
	@[ -f out/avatap.so ] || { echo "run 'make docker' first"; exit 1; }
	$(SSH) 'mkdir -p $(REMOTE_DIR)'
	tar -C out -cf - avatap.so avatap-relay | $(SSH) 'tar -C $(REMOTE_DIR) -xf -'
	tar -cf - inject.sh | $(SSH) 'tar -C $(REMOTE_DIR) -xf - && chmod +x $(REMOTE_DIR)/inject.sh $(REMOTE_DIR)/avatap-relay'
	@$(SSH) 'ls -la $(REMOTE_DIR)'

.PHONY: inject
inject: ## Inject the tap into ava + start the relay (RESTARTS ava; robot idle on dock)
	@echo ">> This restarts ava. Robot should be idle on the dock."
	$(SSH) 'REMOTE_DIR=$(REMOTE_DIR) sh $(REMOTE_DIR)/inject.sh install'

.PHONY: remove
remove: ## Remove the tap, stop the relay, restore camera/stock ava
	$(SSH) 'REMOTE_DIR=$(REMOTE_DIR) sh $(REMOTE_DIR)/inject.sh remove' || true

.PHONY: status
status: ## Show tap + relay state on the robot
	@$(SSH) 'REMOTE_DIR=$(REMOTE_DIR) sh $(REMOTE_DIR)/inject.sh status'

.PHONY: decode
decode: ## Decode the live MCU telemetry stream (odometry/IMU/battery/triggers)
	cargo run -q -p w10-decode -- $(HOST) --mcu

.PHONY: decode-tx
decode-tx: ## Decode the commands ava sends to the MCU (MotorCtrl/SetCleaning/LED)
	cargo run -q -p w10-decode -- $(HOST) --mcu-tx

.PHONY: lds
lds: ## Decode the live LDS scan stream (speed/sector/points/distance; --raw to hexdump)
	cargo run -q -p w10-decode -- $(HOST) --lds

.PHONY: relay-log
relay-log: ## Tail the relay log on the robot
	@$(SSH) 'tail -n 40 /data/log/avatap-relay.log 2>/dev/null'

.PHONY: clean
clean: ## Remove local build artifacts
	cargo clean
	rm -rf out
