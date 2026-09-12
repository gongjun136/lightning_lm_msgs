pipeline {
    agent any

    environment {
        HARBOR_ADDR = '10.233.88.6:60001'
        HARBOR_PROJECT = 'geacx2_aarch64'
        APP_NAME = 'ros2_msg_lib'
        BASE_IMAGE = "${HARBOR_ADDR}/${HARBOR_PROJECT}/ubuntu22.04:latest"
        CONTAINER_NAME = "msg_build_${BUILD_NUMBER}"
        IMAGE_TAG = "${env.GIT_COMMIT.substring(0, 7)}"
        FULL_IMAGE = "${HARBOR_ADDR}/${HARBOR_PROJECT}/${APP_NAME}:${IMAGE_TAG}"
    }

    stages {
        stage('容器内编译ROS2 message包') {
            steps {
                sh '''
                    docker run --rm \
                      --privileged \
                      --name ${CONTAINER_NAME} \
                      -v ${WORKSPACE}:/home/sany/work \
                      --net host \
                      --shm-size 512MB \
                      -w /home/sany/work \
                      -e BUILD_ARCH=aarch64 \
                      ${BASE_IMAGE} \
                      bash -c "
                        set -e
                        # 加载ROS2 humble环境（关键修复）
                        source /opt/ros/humble/setup.bash
                        rm -rf build install
                        colcon build
                        echo '==== msg包编译完成 ===='
                      "
                '''
                echo "消息包编译完成"
            }
        }

        stage('构建消息库镜像') {
            steps {
                sh """
                    docker build -f ci/Dockerfile_msgs -t ${FULL_IMAGE} .
                    docker tag ${FULL_IMAGE} ${HARBOR_ADDR}/${HARBOR_PROJECT}/${APP_NAME}:latest
                    echo "镜像构建成功：${FULL_IMAGE}"
                """
            }
        }

        stage('推送镜像到Harbor') {
            steps {
                withCredentials([usernamePassword(
                    credentialsId: 'harbor-robot-cred',
                    usernameVariable: 'HARBOR_USER',
                    passwordVariable: 'HARBOR_PASS'
                )]) {
                    sh '''
                        docker login ${HARBOR_ADDR} -u ${HARBOR_USER} -p ${HARBOR_PASS}
                        docker push ${FULL_IMAGE}
                        docker push ${HARBOR_ADDR}/${HARBOR_PROJECT}/${APP_NAME}:latest
                        docker logout ${HARBOR_ADDR}
                    '''
                }
            }
        }

        stage('本地镜像清理') {
            steps {
                sh """
                    docker rmi -f ${FULL_IMAGE} ${HARBOR_ADDR}/${HARBOR_PROJECT}/${APP_NAME}:latest || true
                    docker image prune -f
                    cleanWs()
                """
            }
        }
    }

    post {
        failure {
            sh "docker rm -f ${CONTAINER_NAME} || true"
            archiveArtifacts artifacts: 'build/**/*.log', allowEmptyArchive: true
            echo "==== 流水线失败 ===="
        }
        success {
            echo "==== message包CI流水线全部成功，镜像推送到Harbor ===="
        }
    }
}
