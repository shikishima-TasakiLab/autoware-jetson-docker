# Autoware-Jetson
AutowareのJetson用Dockerイメージを作成する．
また，コンテナ上でAutowareの起動を行えるようにする．

## インストール
```bash
#!/bin/bash
git clone --recursive https://github.com/shikishima-TasakiLab/autoware-jetson-docker.git Autoware-Jetson
```

## 使い方

### Dockerイメージの作成

次のコマンドでDockerイメージをビルドする．
```bash
#!/bin/bash
./Autoware-Jetson/docker/build-docker.sh
```
|オプション       |パラメータ|説明                                |既定値  |例         |
|-----------------|----------|------------------------------------|--------|-----------|
|`-h`, `--help`   |なし      |ヘルプを表示                        |なし    |`-h`       |
|`-v`, `--version`|VERSION   |Autowareのバージョンを指定(>=1.12.0)|`1.13.0`|`-v 1.12.0`|

### Dockerコンテナの起動

1. 次のコマンドでDockerコンテナを起動する．
    ```bash
    #!/bin/bash
    ./Autoware-Jetson/docker/run-docker.sh
    ```
    |オプション       |パラメータ|説明                                |既定値    |例                  |
    |-----------------|----------|---------------------------------|----------|--------------------|
    |`-h`, `--help`   |なし      |ヘルプを表示                       |なし      |`-h`                |
    |`-l`, `--launch` |{on\|off} |runtime_managerの起動             |`on`      |`-l off`            |
    |`-p`, `--param`  |FILE      |読み込むAutowareの設定ファイルを指定|`./docker/autoware-param/param_init.yaml`|`-p robot_1.yaml`|
    |`-s`, `--save`   |FILE      |Autowareの設定ファイルの保存先を指定|なし      |`-s robot_1.yaml`|
    |`-n`, `--name`   |NAME      |コンテナの名前を指定               |`autoware`|`-n autoware-master`|

2. AutowareのDockerコンテナで他のROSノードを使用する際は，次のコマンドを別のターミナルで実行する．
    ```bash
    #!/bin/bash
    ./Autoware-Jetson/docker/exec-docker.sh
    ```
    |オプション       |パラメータ|説明                      |既定値|例                  |
    |-----------------|----------|--------------------------|------|--------------------|
    |`-h`, `--help`   |なし      |ヘルプを表示              |なし  |`-h`                |

