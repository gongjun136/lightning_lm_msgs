pipeline {
    agent any
    environment {
        // 定义消息包产物目录
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
# 自动创建目标目录，不存在则新建
# mkdir -p ${MSG_INSTALL_DIR}

# 编译 geosun_msgs，产物输出到指定目录
colcon build --packages-select geosun_msgs --install-base ${MSG_INSTALL_DIR}

# source 环境，自动加载CMAKE_PREFIX_PATH
source ${MSG_INSTALL_DIR}/setup.bash

# 手动追加目录到CMAKE_PREFIX_PATH，兜底，给后续control编译使用
export CMAKE_PREFIX_PATH=${MSG_INSTALL_DIR}:\$CMAKE_PREFIX_PATH

# 验证：检查是否生成 geosun_msgsConfig.cmake
find ${MSG_INSTALL_DIR} -name "geosun_msgsConfig.cmake"
echo "CMAKE_PREFIX_PATH = \$CMAKE_PREFIX_PATH"
"""
            }
        }

        stage('产物校验') {
            steps {
                sh """
set -e
# 校验目录和包文件是否存在
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
