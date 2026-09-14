pipeline {
    agent any
    environment {
        GIT_COMMIT_SHORT = sh(script: 'git rev-parse --short HEAD', returnStdout: true).trim()
        SONAR_TOKEN = credentials('jenkins-sonar')
    }
    stages {
        stage('容器内编译ROS2 message包') {
            steps {
                sh """
                set -e
                echo "==== Jenkins宿主机WORKSPACE = ${WORKSPACE}"
                pwd
                ls -la

                # 启动编译容器，挂载workspace到容器 /home/sany/work
                docker run --rm --privileged --name msg_build_7 \\
                -v ${WORKSPACE}:/home/sany/work \\
                --net host --shm-size 512MB \\
                -w /home/sany/work \\
                -e BUILD_ARCH=aarch64 \\
                10.233.88.6:60001/geacx2_aarch64/ubuntu22.04:latest \\
                bash -c "
                set -e
                source /opt/ros/humble/setup.bash
                rm -rf build install
                colcon build
                echo '==== msg包编译完成 ===='

                # 【修复】先删除旧的Package/install，避免目录已存在报错
                rm -rf /home/sany/work/Package/install
                mkdir -p /home/sany/work/Package
                mv /home/sany/work/install /home/sany/work/Package/install
                ls -la /home/sany/work/Package

                # 执行rename_msgs.sh脚本

                echo '==== 开始执行 rename_msgs.sh ===='
                rename_msgs.sh
                echo '==== rename_msgs.sh 执行完成 ===='
                "
                """
            }
        }

        // ========== Sonar扫描，捕获CE TaskId，轮询质量门禁 ==========
        stage('SonarQube 代码扫描') {
            steps {
                withSonarQubeEnv('SonarQube') {
                    sh '''
                    sonar-scanner \
                        -Dsonar.projectKey=my-project \
                        -Dsonar.projectName=ROS2-Msg-Package \
                        -Dsonar.projectVersion=${GIT_COMMIT_SHORT} \
                        -Dsonar.sources=. \
                        -Dsonar.exclusions=build/**,install/**,Package/**,**/*.md,**/*.swp \
                        -Dsonar.host.url=http://10.233.88.16:9000 \
                        -Dsonar.token=${SONAR_TOKEN} > sonar_out.log 2>&1
                    '''
                    
                }
            }
        }

        stage('本地镜像清理') {
            steps {
                sh """
                docker image prune -f
                """
            }
        }
    }
    post {
        always {
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
