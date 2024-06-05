#!/bin/bash

mkdir -p artifacts
for rev in GIZMO_VERSION_R3B GIZMO_VERSION_R4B GIZMO_VERSION_R6E ; do
    arduino-cli compile --profile gizmo --build-property "build.extra_flags=-D $rev" --output-dir out .
    cp out/firmware.ino.uf2 "artifacts/gss_${rev/GIZMO_VERSION_/}_${GITHUB_REF#refs/*/}.uf2"
done
