pipeline {
    agent any
    environment {
        MSG_INSTALL_DIR = "/home/sany/work/Package/Common/message"
    }
    stages {
        stage('Checkout SCM') {
            steps {
                checkout scm
            }
        }

        stage('容器内编译ROS2 message包') {
            steps {
                sh """
set -e
# 加载ROS2环境，关键修复
source /opt/ros/humble/setup.bash

# mkdir -p ${MSG_INSTALL_DIR}

colcon build --packages-select geosun_msgs --install-base ${MSG_INSTALL_DIR}

# 加载编译出来的消息包环境
source ${MSG_INSTALL_DIR}/setup.bash
export CMAKE_PREFIX_PATH=${MSG_INSTALL_DIR}:\$CMAKE_PREFIX_PATH

find ${MSG_INSTALL_DIR} -name "geosun_msgsConfig.cmake"
echo "CMAKE_PREFIX_PATH = \$CMAKE_PREFIX_PATH"
"""
            }
        }

        stage('产物校验') {
            steps {
                sh """
set -e
source /opt/ros/humble/setup.bash
test -d ${MSG_INSTALL_DIR}
find ${MSG_INSTALL_DIR}/share/geosun_msgs -name "geosun_msgsConfig.cmake"
"""
            }
        }
    }

    post {
        success {
            echo "message包编译完成，产物路径：${MSG_INSTALL_DIR}"
        }
        failure {
            echo "message包编译失败！"
        }
    }
}
