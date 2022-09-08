.PHONY: help plane rc

help:
	@echo "make plane: Build plane firmware."
	@echo "make rc: Build remote control firmware."

plane:
	cd plane; \
	arduino --upload plane.ino

rc:
	cd rc; \
	arduino --upload rc.ino
