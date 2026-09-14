pipeline {
    agent any
    environment {
        // 取git短commit hash
        GIT_COMMIT_SHORT = sh(script: 'git rev-parse --short HEAD', returnStdout: true).trim()
    }
    stages {
        stage('容器内编译ROS2 message包') {
            sh '''
set -e
echo "==== Jenkins宿主机WORKSPACE = ${WORKSPACE}"
pwd
ls -la

# 启动编译容器，挂载workspace到容器 /home/sany/work
docker run --rm --privileged --name msg_build_7 \
-v ${WORKSPACE}:/home/sany/work \
--net host --shm-size 512MB \
-w /home/sany/work \
-e BUILD_ARCH=aarch64 \
10.233.88.6:60001/geacx2_aarch64/ubuntu22.04:latest \
bash -c "
set -e
source /opt/ros/humble/setup.bash
rm -rf build install
colcon build
echo '==== msg包编译完成 ===='

# 新建Package目录，将编译产物install移入Package/install
mkdir -p /home/sany/work/Package
mv /home/sany/work/install /home/sany/work/Package/install
ls -la /home/sany/work/Package

# 执行rename_msgs.sh脚本
chmod +x /home/sany/work/rename_msgs.sh
echo '==== 开始执行 rename_msgs.sh ===='
/home/sany/work/rename_msgs.sh
echo '==== rename_msgs.sh 执行完成 ===='
"
'''
        }

        stage('本地镜像清理') {
            sh '''
docker image prune -f
'''
        }
    }
    post {
        always {
            // 无论成功失败，清理残留构建容器
            sh 'docker rm -f msg_build_7 || true'
        }
        success {
            echo "✅ 流水线全部执行成功！产物目录：${WORKSPACE}/Package/install"
        }
        failure {
            echo "❌ 流水线执行失败，请查看日志排查"
        }
    }
}
