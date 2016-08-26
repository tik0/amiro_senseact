#!/bin/sh

if [ -z "${1}" ]; then
  echo "Set an ID for the AMiRo, which shall be corrected"
  exit 1
fi

ID=${1}

echo "Correct AMiRo with ID ${ID}"

./choreoCorrection -a ${ID} -m 18


