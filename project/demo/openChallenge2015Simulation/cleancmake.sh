#!/bin/sh

for line in `ls -d */ | sed 's#\ ##g' | sed 's#\/##g' | grep -v CMakeFiles`; do
  data=$(ls $line | sed 's#\ ##g' | sed 's#\/##g' | grep $line);
  if [ "$data" = "$line" ]; then
    rm -rf $line
  fi
done
rm *~
rmcmake

