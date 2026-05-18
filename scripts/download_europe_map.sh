#!/usr/bin/env bash
set -euo pipefail

mkdir -p data/raw
cd data/raw

file="europe-260513.osm.pbf"
url="https://download.geofabrik.de/${file}"

curl -C - -L --fail --retry 3 --output "$file" "$url"
echo "downloaded: $PWD/$file"
ls -lh "$file"
