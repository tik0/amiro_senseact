#!/bin/bash

# check if both parameters are given
if [ -z "${1}" ]; then
  echo "Give first the old path and then the new path to the moved project!"
  exit 1
fi

if [ -z "${2}" ]; then
  echo "The new path to the project is missed!"
  exit 1
fi

# load actual and start directory
oldDirectory=$(pwd)
startDirectory=$MUROX_PROJECT

# prepare old project's path and reference string for sed
oldArr=$(echo $1 | tr "/" "\n")
oldPath="}"
oldRef="\[\["
oldProject=""
for x in $oldArr; do
  if [[ ${#x} -gt 0 ]]; then
    oldPath=$(echo "$oldPath\/$x")
    oldRef=$(echo "$oldRef$x:")
    oldProject=$(echo "$oldProject$x/")
  fi
done

# prepare new project's path and reference string for sed
newArr=$(echo $2 | tr "/" "\n")
newPath="}"
newRef="\[\["
newProject=""
for x in $newArr; do
  if [[ ${#x} -gt 0 ]]; then
    newPath=$(echo "$newPath\/$x")
    newRef=$(echo "$newRef$x:")
    newProject=$(echo "$newProject$x/")
  fi
done

# check if new project exists
fullProjectPath=$(echo "$startDirectory$newProject")
if [ -d "$fullProjectPath" ]; then
  echo "Replace old project $oldProject by new project $newProject."
else
  echo "The new project $fullProjectPath doesn't exist! There aren't any replacements."
  exit 1
fi

#echo "Old project: $oldProject - $oldPath - $oldRef"
#echo "New project: $newProject - $newPath - $newRef"

# define function checking directories recursively
checkDirectory () {
  for d in *; do
    # check for subdirectory
    if [ -d $d ]; then
      cd "$d"
      checkDirectory
      cd ..

    # check for CMakeLists
    elif [[ $d == CMakeLists.txt ]]; then
      sed -i "s/$oldPath/$newPath/g" $d

    # check for txt file
    elif [[ $d == *.txt ]]; then
      sed -i "s/$oldRef/$newRef/g" $d
    fi
  done
  rm -f *~
}

# change to main project directory
cd $startDirectory

# check all subdirectories
checkDirectory

# change back
cd $oldDirectory

echo "Replacement done."

