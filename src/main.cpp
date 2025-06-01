#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/algorithm/string.hpp> // For case-insensitive string comparison
#include <pcl/filters/extract_indices.h> // Required for pcl::ExtractIndices

// 設定ファイルのパス (固定)
const std::string CONFIG_FILE_PATH = "../config/config.txt"; // ビルドディレクトリからの相対パスを想定

// 設定パラメータを保持する構造体
struct ConfigParams {
    // ProgressiveMorphologicalFilter Parameters
    int max_window_size = 33;
    float slope = 1.0f;
    float initial_distance = 0.15f;
    float max_distance = 3.0f;

    // Downsampling Parameters
    float voxel_leaf_size = 0.1f;
};

// 設定ファイルを読み込む関数
bool loadConfig(ConfigParams& params) {
    std::ifstream configFile(CONFIG_FILE_PATH);
    if (!configFile.is_open()) {
        std::cerr << "エラー: 設定ファイルを開けませんでした: " << CONFIG_FILE_PATH << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(configFile, line)) {
        // コメント行と空行をスキップ
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        std::string key, value;
        if (std::getline(iss, key, '=') && std::getline(iss, value)) {
            // 前後の空白をトリム
            boost::trim(key);
            boost::trim(value);

            try {
                if (key == "max_window_size") {
                    params.max_window_size = std::stoi(value);
                } else if (key == "slope") {
                    params.slope = std::stof(value);
                } else if (key == "initial_distance") {
                    params.initial_distance = std::stof(value);
                } else if (key == "max_distance") {
                    params.max_distance = std::stof(value);
                } else if (key == "voxel_leaf_size") {
                    params.voxel_leaf_size = std::stof(value);
                }
            } catch (const std::invalid_argument& e) {
                std::cerr << "警告: 設定値の変換に失敗しました: " << key << " = " << value << " (" << e.what() << ")" << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "警告: 設定値が範囲外です: " << key << " = " << value << " (" << e.what() << ")" << std::endl;
            }
        }
    }
    configFile.close();
    std::cout << "設定ファイルを読み込みました。" << std::endl;
    std::cout << " - max_window_size: " << params.max_window_size << std::endl;
    std::cout << " - slope: " << params.slope << std::endl;
    std::cout << " - initial_distance: " << params.initial_distance << std::endl;
    std::cout << " - max_distance: " << params.max_distance << std::endl;
    std::cout << " - voxel_leaf_size: " << params.voxel_leaf_size << std::endl;
    return true;
}

// PCLVisualizerで点群を表示する関数
void visualizeCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                    const std::string& window_title,
                    int r = 255, int g = 255, int b = 255, // デフォルトは白
                    const std::string& cloud_id = "cloud") {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_title));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, r, g, b);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, cloud_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_id);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

// PCLVisualizerで2つの点群を色分けして表示する関数
void visualizeGroundAndObstacles(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ground_cloud,
                                 const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obstacle_cloud,
                                 const std::string& window_title) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_title));
    viewer->setBackgroundColor(0, 0, 0);

    // 地面を緑で表示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ground_color_handler(ground_cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(ground_cloud, ground_color_handler, "ground_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground_cloud");

    // 障害物を赤で表示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> obstacle_color_handler(obstacle_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(obstacle_cloud, obstacle_color_handler, "obstacle_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "obstacle_cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}


int main(int argc, char** argv) {
    // コマンドライン引数のチェック
    if (argc != 2) {
        std::cerr << "使用方法: " << argv[0] << " <input_point_cloud_file>" << std::endl;
        std::cerr << "対応ファイル形式: .pcd, .ply" << std::endl;
        return -1;
    }

    std::string inputFile = argv[1];
    std::string extension = "";
    size_t pos = inputFile.rfind('.');
    if (pos != std::string::npos) {
        extension = inputFile.substr(pos);
    }

    // ファイル形式の検証
    bool is_pcd = boost::iequals(extension, ".pcd");
    bool is_ply = boost::iequals(extension, ".ply");

    if (!is_pcd && !is_ply) {
        std::cerr << "エラー: 未対応のファイル形式です: " << extension << std::endl;
        std::cerr << "PCD (.pcd) または PLY (.ply) ファイルを指定してください。" << std::endl;
        std::cerr << "読み込み方法:" << std::endl;
        std::cerr << "  PCD: pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);" << std::endl;
        std::cerr << "  PLY: pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud);" << std::endl;
        return -1;
    }

    // 設定ファイルの読み込み
    ConfigParams params;
    if (!loadConfig(params)) {
        std::cerr << "設定ファイルの読み込みに失敗しました。デフォルト値で続行します。" << std::endl;
        // デフォルト値は構造体初期化時に設定済み
    }

    // 点群オブジェクトの作成
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 点群ファイルの読み込み
    std::cout << inputFile << " を読み込んでいます..." << std::endl;
    if (is_pcd) {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputFile, *cloud) == -1) {
            std::cerr << "エラー: PCDファイルの読み込みに失敗しました: " << inputFile << std::endl;
            return -1;
        }
    } else if (is_ply) {
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(inputFile, *cloud) == -1) {
            std::cerr << "エラー: PLYファイルの読み込みに失敗しました: " << inputFile << std::endl;
            return -1;
        }
    }
    std::cout << "読み込み完了。 " << cloud->width * cloud->height << " 点のデータを読み込みました。" << std::endl;

    // 処理前の点群を表示
    std::cout << "処理前の点群を表示します..." << std::endl;
    visualizeCloud(cloud, "Original Cloud");

    // ダウンサンプリング
    if (params.voxel_leaf_size > 0) {
        std::cout << "ダウンサンプリングを実行中 (Voxel Leaf Size: " << params.voxel_leaf_size << ")..." << std::endl;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(params.voxel_leaf_size, params.voxel_leaf_size, params.voxel_leaf_size);
        sor.filter(*cloud_downsampled);
        std::cout << "ダウンサンプリング完了。 " << cloud_downsampled->width * cloud_downsampled->height << " 点になりました。" << std::endl;
    } else {
        std::cout << "voxel_leaf_size <= 0 のため、ダウンサンプリングをスキップします。" << std::endl;
        *cloud_downsampled = *cloud; // ダウンサンプリングしない場合は元の点群をコピー
    }


    // Progressive Morphological Filter を使って地面を判定
    std::cout << "Progressive Morphological Filter を実行中..." << std::endl;
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(cloud_downsampled);
    pmf.setMaxWindowSize(params.max_window_size);
    pmf.setSlope(params.slope);
    pmf.setInitialDistance(params.initial_distance);
    pmf.setMaxDistance(params.max_distance);

    pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
    pmf.extract(ground_indices); // 地面のインデックスを取得

    // 地面とそれ以外（障害物）を分離
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_downsampled);
    extract.setIndices(ground_indices);

    extract.setNegative(false); // false = 地面を抽出
    extract.filter(*ground_cloud);
    std::cout << "地面の点群数: " << ground_cloud->size() << std::endl;

    extract.setNegative(true); // true = 地面以外（障害物）を抽出
    extract.filter(*obstacle_cloud);
    std::cout << "障害物の点群数: " << obstacle_cloud->size() << std::endl;
    std::cout << "地面の判定完了。" << std::endl;

    // 地面（緑）と障害物（赤）を表示
    std::cout << "処理後の点群を表示します (地面: 緑, 障害物: 赤)..." << std::endl;
    visualizeGroundAndObstacles(ground_cloud, obstacle_cloud, "Segmented Cloud (Ground: Green, Obstacles: Red)");

    // 障害物のみの点群をPCDファイルとして出力
    if (!obstacle_cloud->points.empty()) {
        std::string output_filename = "filtered_cloud.pcd";
        pcl::io::savePCDFileASCII(output_filename, *obstacle_cloud);
        std::cout << "障害物の点群を " << output_filename << " として保存しました。" << std::endl;
    } else {
        std::cout << "障害物が検出されなかったため、ファイルは出力されませんでした。" << std::endl;
    }

    std::cout << "処理が完了しました。" << std::endl;

    return 0;
}
