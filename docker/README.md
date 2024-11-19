**Updating Docker Image:**

If you need to change or update the docker images for the Lunabotics project, start by making your changes to your local Dockerfiles. Make any changes you need here, and push these changes to GitHub. Make sure that the changes you are making are relevant to the docker file that you’re adding it to. If it isn’t, either pick a different file or make a new dockerfile (if you make a new one, make sure to add the new dockerfile name to the image_key in /scripts/enter_isaac_ros_container.sh and /scripts/build_image.sh). As of right now, we’re keeping all of our docker files in Lunabotics/docker and the Nvidia’s files in Lunabotics/src/isaac_ros_common.

After making your changes locally, build them with `./scripts/enter_isaac_ros_container false`
This will run all of the local dockerfiles without defaulting to the cached image on dockerhub. 
After building and opening this container, run tests to make sure that this image doesn’t break anything. 
Find the highest level image of this build with docker image list. The highest level image will be the largest image shown by docker image list, and will likely be isaac_ros_dev-x86-64. 
Tag this image as `umnrobotics/<repo-name>:<image-layers-separated-by->_<HASH>`
At time of writing, we were tagging images as 'umnrobotics/isaac_ros3.1:x86_64-ros2_humble-realsense-deepstream-user-zed-umn-gazebo_d000f8df5f3859fd56c7459b2ad3a718'


If you are logged in to docker, running docker image push will push this image to the cloud, and you are done. If you changed the name of/created a new dockerhub repo, update BASE_DOCKER_REGISTRY_NAMES in /scripts/build_image.sh to reflect the new name. 

After doing this, everything should be set up. Running ./scripts/enter_isaac_ros_container should default to the newly cached image, with all of the new changes. 
When downloading the docker image on a separate device for the first time use “./scripts/enter_isaac_ros_container cached” to pull the new image. If it doesn’t do this, you may need to run docker image rm <old image name> to ensure that it pulls the new image.

**Basic Docker Commands:**


`docker login` – Prompts you to login to dockerhub. If maintaining team docker images, use the umnrobotics account. 

`docker image list` – Shows all docker images that are built on your device

`docker ps – lists alls RUNNING` images

`docker kill <name (press tab)>` – Kills the specified image, stops it from running

`docker image rm <name (press tab)>` – deletes the specified image from your computer

`docker image prune` – Deletes headless / unused images. These images show as ‘<none>’ when running `docker image list

`docker system prune -a --volumes` – Deletes and clears ALL built images from the container

`docker image tag <local image tag> <new tag name>` – Essentially just renames a docker image

`docker image push <image name>` – Pushes local image to the cloud where it can be downloaded and used by others.
