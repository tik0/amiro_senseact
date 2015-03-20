#!/bin/bash
echo "Running protoc for every *.proto file to create the *pb.cc and *pb.h files:"
# To be more explicit, we run every file to get propter output
for list in `ls *.proto`
do
  echo "protoc ${list} --cpp_out=."
  protoc ${list} --cpp_out=. --proto_path=. --proto_path=/usr/local/share/rst0.11/proto/stable/ --proto_path=/usr/local/share/rst0.11/proto/sandbox/
done
