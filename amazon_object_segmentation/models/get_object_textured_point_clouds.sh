#!/bin/sh

echo "This will download the amazon objects point clouds. You need 1.7GB for this."

wget https://www.dropbox.com/s/b74co6v785scj0r/amazon_object_textured_meshes.tar.gz?dl=0
tar -xvf amazon_object_textured_meshes.tar.gz?dl=0
rm -i amazon_object_textured_meshes.tar.gz?dl=0

echo "Point clouds of objects installed. Check ./amazon_object_textured_meshes"
