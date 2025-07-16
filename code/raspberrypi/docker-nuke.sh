#!/bin/bash

echo "Stopping all running containers..."
sudo docker stop $(sudo docker ps -q)

echo "Removing all containers..."
sudo docker rm $(sudo docker ps -a -q)

echo "Removing all images..."
sudo docker rmi -f $(sudo docker images -q)

echo "Removing all volumes..."
sudo docker volume rm $(sudo docker volume ls -q)

echo "Removing all user-defined networks (except default ones)..."
sudo docker network rm $(sudo docker network ls | grep -v "bridge\|host\|none" | awk '{print $1}')

echo "Docker cleanup complete. Current Docker info:"
sudo docker system prune -af --volumes
sudo docker info