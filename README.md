# PCLを使った障害物抽出プログラム

## 概要

このプログラムは、PCD (Point Cloud Data) または PLY 形式の点群ファイルを読み込み、PCL (Point Cloud Library) の Progressive Morphological Filter を使用して地面と障害物を分離し、障害物のみの点群ファイルを出力します。また、処理の各段階で点群をPCLViewerで可視化します。

## 主な機能

*   PCDまたはPLYファイルの読み込み
*   設定ファイル (`config/config.txt`) によるパラメータ調整
    *   ダウンサンプリングのボクセルサイズ
    *   Progressive Morphological Filter の各種パラメータ
*   処理前の点群の表示
*   点群のダウンサンプリング
*   Progressive Morphological Filter による地面と障害物の分離
*   地面（緑色）と障害物（赤色）の色分け表示
*   障害物のみの点群 (`filtered_cloud.pcd`) の出力

## 動作環境

*   OS: Ubuntu 22.04 / 24.04 (WindowsやMacOSは非対応)
*   PCL (Point Cloud Library)

## インストール方法

1.  **リポジトリのクローン:**
    ```bash
    git clone <リポジトリのURL>
    cd <リポジトリ名>
    ```

2.  **PCLのインストール:**
    リポジトリのルートディレクトリにある `install_pcl.sh` スクリプトを実行して、PCLライブラリと関連ツールをインストールします。
    ```bash
    chmod +x install_pcl.sh
    ./install_pcl.sh
    ```
    このスクリプトは `apt-get` を使用してPCLをインストールします。

## ビルドの方法

1.  **buildディレクトリの作成と移動:**
    ```bash
    mkdir build
    cd build
    ```

2.  **CMakeの実行とビルド:**
    `build` ディレクトリ内で以下のコマンドを実行して、プロジェクトをビルドします。
    ```bash
    cmake ..
    make
    ```
    ビルドが成功すると、`build` ディレクトリ内に `pcl_obstacle_detection` という名前の実行ファイルが生成されます。

## プログラムの使用方法

1.  **点群ファイルの準備:**
    処理したいPCDまたはPLY形式の点群ファイルを `data` ディレクトリなど、任意の場所に配置します。

2.  **設定ファイルの編集 (任意):**
    必要に応じて `config/config.txt` ファイル内のパラメータを編集します。
    ```txt
    # config/config.txt の例
    # ProgressiveMorphologicalFilter Parameters
    max_window_size = 33  # 最大ウィンドウサイズ
    slope = 1.0           # 地面の傾斜の許容値
    initial_distance = 0.15 # 地面と判断する初期の高さ閾値
    max_distance = 3.0    # 地面と判断する最大の高さ閾値

    # Downsampling Parameters
    voxel_leaf_size = 0.1 # ダウンサンプリング時のボクセルサイズ (0以下でスキップ)
    ```

3.  **プログラムの実行:**
    `build` ディレクトリから、コマンドライン引数に点群ファイルのパスを指定してプログラムを実行します。
    ```bash
    ./pcl_obstacle_detection <点群ファイルのパス>
    ```
    例:
    ```bash
    ./pcl_obstacle_detection ../data/your_point_cloud.pcd
    # または
    ./pcl_obstacle_detection ../data/your_point_cloud.ply
    ```

4.  **結果の確認:**
    *   プログラム実行中に、PCLViewerが起動し、処理前後の点群が表示されます。
        *   最初のウィンドウ: 読み込んだ生の点群
        *   次のウィンドウ: 地面（緑色）と障害物（赤色）に分離された点群
    *   処理が完了すると、`build` ディレクトリ（実行ファイルと同じ場所）に `filtered_cloud.pcd` という名前で障害物のみの点群が出力されます。

## ディレクトリ構成

```
.
├── CMakeLists.txt              # CMakeビルド設定
├── README.md                   # このファイル
├── build/                      # ビルドファイル格納ディレクトリ (git管理外)
├── config/
│   └── config.txt              # パラメータ設定ファイル
├── data/                       # 点群データ格納用 (空の状態で提供)
├── install_pcl.sh              # PCLインストールスクリプト
└── src/
    └── main.cpp                # C++ソースコード
```

## その他

*   エラーメッセージや処理状況は標準出力に表示されます。
*   PCLViewerのウィンドウを閉じることで、次の処理に進んだり、プログラムを終了したりできます。
```
