pipeline {
  agent any
  environment {
    TIVA_WARE_PATH = '/opt/tivaware'
  }
  stages {
    stage('Build') {
      steps {
        sh 'printenv'
        dir(path: 'catkin_ws') {
          sh '''
            . /opt/ros/kinetic/setup.sh
            catkin build --no-status --verbose
          '''
        }
      }
    }
    stage('Test') {
      steps {
        dir(path: 'catkin_ws') {
          sh '''
            . /opt/ros/kinetic/setup.sh
            . devel/setup.sh
            catkin run_tests
            catkin_test_results build --verbose
          '''
        }
      }
    }
    stage('Sanity Check') {
      parallel {
        stage('Lint') {
          steps {
            sh '''
              . /opt/ros/kinetic/setup.sh
              catkin lint --explain --strict -W2 catkin_ws/src \
                --ignore target_name_collision \
                --ignore missing_install_target \
                --ignore critical_var_append \
                --ignore link_directory \
                --skip-pkg rosserial_tivac \
                --skip-pkg gscam
            '''
          }
        }
      }
    }
  }
  post {
    always {
      archiveArtifacts(artifacts: 'catkin_ws/logs/**/*.log', fingerprint: true)

      script {
        def files = findFiles glob: 'catkin_ws/build/**/test_results/**/*.xml'
        if (files.length > 0) {
          junit 'catkin_ws/build/**/test_results/**/*.xml'
        }
      }
    }
  }
}
