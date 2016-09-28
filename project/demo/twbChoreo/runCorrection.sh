#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set the robot ID for the AMiRo, which shall be corrected"
  exit 1
fi

if [ -z "${2}" ]; then
  echo "Set the marker ID for the AMiRo, which shall be corrected"
  exit 1
fi

ID=${1}
MARKER=${2}

echo "Correct AMiRo with ID ${ID} and marker ${MARKER}"

./choreoCorrection -a ${ID} -m ${MARKER}


