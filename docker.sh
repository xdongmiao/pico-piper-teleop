#!/bin/bash

CONTAINER_NAME="xr"

# 检查容器是否存在（包括已停止的）
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
    # 容器存在
    echo "容器 '${CONTAINER_NAME}' 已存在."
    
    # 检查容器是否正在运行
    if [ "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
        # 容器正在运行，直接进入
        echo "容器正在运行，进入容器..."
        docker exec -it ${CONTAINER_NAME} bash
    else
        # 容器已停止，先启动再进入
        echo "容器未运行，启动容器..."
        docker start ${CONTAINER_NAME}
        echo "进入容器..."
        docker exec -it ${CONTAINER_NAME} bash
    fi
else
    # 容器不存在，创建并运行
    echo "容器不存在 '${CONTAINER_NAME}'..."
fi
