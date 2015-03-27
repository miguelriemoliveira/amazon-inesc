#!/bin/sh

echo "This will download the amazon objects point clouds. You need 1.7GB for this."
wget https://www.dropbox.com/s/zeq5k82wj6exusw/amazon_object_textured_point_clouds.tar.gz?dl=0
tar -xvf amazon_object_textured_point_clouds.tar.gz?dl=0
rm -i amazon_object_textured_point_clouds.tar.gz?dl=0

echo "Point clouds of objects installed. Check ./amazon_object_textured_point_clouds"
