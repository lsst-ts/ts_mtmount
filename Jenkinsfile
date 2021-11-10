properties(
    [
    buildDiscarder
        (logRotator (
            artifactDaysToKeepStr: '',
            artifactNumToKeepStr: '',
            daysToKeepStr: '14',
            numToKeepStr: '10'
        ) ),
    disableConcurrentBuilds()
    ]
)

pipeline {

    agent {
        // Use the docker to assign the Python version.
        // Use the label to assign the node to run the test.
        // It is recommended by SQUARE to not add the label
        docker {
            alwaysPull true
            image 'lsstts/develop-env:develop'
            args "-u root --entrypoint=''"
        }
    }

    environment {
        // XML report path
        XML_REPORT="jenkinsReport/report.xml"
        // Module name used in the pytest coverage analysis
        MODULE_NAME="lsst.ts.mtmount"
        user_ci = credentials('lsst-io')
        LTD_USERNAME="${user_ci_USR}"
        LTD_PASSWORD="${user_ci_PSW}"
        work_branches = "${GIT_BRANCH} ${CHANGE_BRANCH} develop"
    }

    stages {
        stage ('Install Requirements And Update Branches') {
            steps {
                // When using the docker container, we need to change
                // the HOME path to WORKSPACE to have the authority
                // to install the packages.
                withEnv(["HOME=${env.WORKSPACE}"]) {
                    sh """
                        source /home/saluser/.setup_dev.sh || echo loading env failed. Continuing...
                        cd /home/saluser/repos/ts_utils
                        /home/saluser/.checkout_repo.sh ${work_branches}
                        git pull
                        cd /home/saluser/repos/ts_xml
                        /home/saluser/.checkout_repo.sh ${work_branches}
                        git pull
                        cd /home/saluser/repos/ts_sal
                        /home/saluser/.checkout_repo.sh ${work_branches}
                        git pull
                        cd /home/saluser/repos/ts_salobj
                        /home/saluser/.checkout_repo.sh ${work_branches}
                        git pull
                        cd /home/saluser/repos/ts_idl
                        /home/saluser/.checkout_repo.sh ${work_branches}
                        git pull
                        cd /home/saluser/repos/ts_config_mttcs
                        /home/saluser/.checkout_repo.sh ${work_branches}
                        git pull
                        cd /home/saluser/repos/ts_simactuators
                        /home/saluser/.checkout_repo.sh ${work_branches}
                        git pull
                        cd /home/saluser/repos/ts_tcpip
                        /home/saluser/.checkout_repo.sh ${work_branches}
                        git pull
                        make_idl_files.py MTMount MTRotator
                    """
                }
            }
        }

        stage('Unit Tests and Coverage Analysis') {
            steps {
                // Direct the HOME to WORKSPACE for pip to get the
                // installed library.
                // 'PATH' can only be updated in a single shell block.
                // We can not update PATH in 'environment' block.
                // Pytest needs to export the junit report.
                withEnv(["HOME=${env.WORKSPACE}"]) {
                    sh """
                        source /home/saluser/.setup_dev.sh || echo loading env failed. Continuing...
                        setup -k -r .
                        pytest --cov-report html --cov=${env.MODULE_NAME} --junitxml=${env.XML_REPORT}
                    """
                }
            }
        }
        stage('Build and Upload Documentation') {
            steps {
                withEnv(["HOME=${env.WORKSPACE}"]) {
                    sh """
                        source /home/saluser/.setup_dev.sh || echo loading env failed. Continuing...
                        pip install .
                        package-docs build
                        ltd upload --product ts-mtmount --git-ref ${GIT_BRANCH} --dir doc/_build/html
                    """
                }
            }
        }
    }

    post {
        always {
            // Change the ownership of workspace to Jenkins for the clean up
            // This is a "work around" method
            withEnv(["HOME=${env.WORKSPACE}"]) {
                sh 'chown -R 1003:1003 ${HOME}/'
            }

            // The path of xml needed by JUnit is relative to
            // the workspace.
            junit 'jenkinsReport/*.xml'

            // Publish the HTML report
            publishHTML (target: [
                allowMissing: false,
                alwaysLinkToLastBuild: false,
                keepAll: true,
                reportDir: 'htmlcov',
                reportFiles: 'index.html',
                reportName: "Coverage Report"
              ])
        }

        cleanup {
            // clean up the workspace
            deleteDir()
        }
    }
}
