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
        stage('SonarQube 代码扫描与质量门禁') {
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
                    // 提取CE task id
                    sh '''
#!/bin/sh
set -e
SONAR_HOST="http://10.233.88.16:9000"
TOKEN="${SONAR_TOKEN}"

# 从sonar输出日志提取CE task id
CE_TASK_ID=$(grep "api/ce/task?id=" sonar_out.log | head -1 | cut -d'=' -f2)
echo "CE_TASK_ID = ${CE_TASK_ID}"

# 轮询CE任务直到SUCCESS
CE_STATUS=""
i=0
while [ $i -lt 30 ]; do
    i=$((i+1))
    echo "Poll CE task, attempt $i"
    CE_RESP=$(curl -s -u "${TOKEN}:" "${SONAR_HOST}/api/ce/task?id=${CE_TASK_ID}")
    CE_STATUS=$(echo "${CE_RESP}" | grep -o '"status":"[^"]*"' | head -1 | cut -d'"' -f4)
    echo "CE status = ${CE_STATUS}"
    if [ "${CE_STATUS}" = "SUCCESS" ]; then
        break
    fi
    if [ "${CE_STATUS}" = "FAILED" ]; then
        echo "❌ Sonar CE任务失败"
        exit 1
    fi
    sleep 3
done

if [ "${CE_STATUS}" != "SUCCESS" ]; then
    echo "⏱️ CE任务超时未完成"
    exit 1
fi

# 拿到analysisId
ANALYSIS_ID=$(echo "${CE_RESP}" | grep -o '"analysisId":"[^"]*"' | head -1 | cut -d'"' -f4)
echo "ANALYSIS_ID = ${ANALYSIS_ID}"

# 查询质量门禁
QG_RESP=$(curl -s -u "${TOKEN}:" "${SONAR_HOST}/api/qualitygates/project_status?analysisId=${ANALYSIS_ID}")
QG_STATUS=$(echo "${QG_RESP}" | grep -o '"status":"[^"]*"' | head -1 | cut -d'"' -f4)
echo "QualityGate status = ${QG_STATUS}"

if [ "${QG_STATUS}" = "OK" ]; then
    echo "✅ Sonar质量门禁校验通过"
    exit 0
else
    echo "❌ Sonar质量门禁校验不通过"
    exit 1
fi
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
