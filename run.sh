#! /bin/bash

echo "Algab!"

dts devel build -f -H shelby.local
dts devel run -H shelby.local -- --privileged

echo "Lõpetab!"
