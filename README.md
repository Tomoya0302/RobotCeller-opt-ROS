# RobotCeller-opt-ROS

## 問題設定
### 構成要素
- 1台のロボット(Niryo Ned)
- 1台の組立テーブル
- 複数の部品収納棚

### 多目的最適化
ロボットセル生産システムの導入時に重要となる，構成要素 (ロボット・テーブル・部品収納棚) の配置を決定します．\
評価関数は，

1. ロボットの動作時間
2. レイアウトの面積

の2つで多目的最適化計算を行います．\
ロボットは，配置された部品収納棚から部品をテーブルに運びます．
ロボットの動作は，\
テーブル → 部品収納棚1 → テーブル → 部品収納棚1 → テーブル → ... としています．\
このとき，ロボットはすべての部品収納棚，テーブルに届くことが制約となります．
また，各構成要素は重ならないように配置する必要があります．

## アルゴリズム
1. シーケンスペアによってレイアウトを生成
2. レイアウトの面積を計算
3. レイアウトに従ってロボットによって動作計画を実施，
4. 実行可能性，ロボットの動作時間を取得
6. NSGA-IIで最適化 (1-4を繰り返す)

## ソースコードの構成
![src](https://github.com/Tomoya0302/RobotCeller-opt-ROS/assets/23186611/5511730a-8c2b-4af9-ae44-377cdfb1a469)

- レイアウトは，```rectangle_packing_solver```内のシーケンスペアアルゴリズムによって生成されます．
- レイアウトの面積は，```layout_generation.py```内の```Layout```クラスで計算します．
- ロボットの動作時間と実行可能性は，```layout_generation.py```内の```Robot```クラスで計算します．
- これらは，ロボットの動作計画によって正確な値を取得するために，```layout_ned.py```とSocket通信を行います．
- ```layout_ned.py```では，MoveIt!によってロボットの動作計画をシミュレーションします．
- NSGA-IIは，```layout_opt.py```で最適化計算を行います．

> 使用するロボットの都合で，```layout_ned.py```のみPython2系で実装しています．

## 環境構築

### Python
Python3系，Python2系の両方が必要です．

- python 3.10.7
- python 2.7.17

で動作確認済みです．

### ライブラリ
Python3では下記バージョンのライブラリが必要です．

| ライブラリ | バージョン |
| ---- | ---- |
| pymoo | 0.6.0 |
| PyYaml | 6.0 |
| Matplotlib | - |

### ROS
ROS melodic での動作を確認済みです．\
Ubuntu 18.04 でサポートされています．

Niryo Nedをシミュレーション環境で使用するために，下記サイトを参考にして環境構築してください．\
https://docs.niryo.com/dev/ros/v4.1.1/en/source/installation/ubuntu_18.html

## 実行手順
1. 新しいターミナルで\
  ```$ roslaunch niryo_robot_bringup desktop_rviz_simulation.launch```

2. 新しいターミナルで\
  ```$ python3 layout_opt.py```

3. 新しいターミナルで\
  ```$ python2 layout_ned.py```


## 各パラメータの変更

### 構成要素の数・サイズ

一覧\
```layout_generation.py``` l.14~

構成要素の数\
```layout_opt.py``` l.16 ```self.n_rect = 12```

### ロボットの動作計画がかなりの確率で失敗する場合

コンピュータの性能によっては動作計画の最大時間を伸ばす必要があります．\
```layout_ned.py``` l.55 ```self.move_group.set_planning_time(0.2)```

### NSGA-IIの世代数・個体数

個体数\
```layout_opt.py``` l.161 ```pop_size=100```

世代数\
```layout_opt.py``` l.169 ```('n_gen', 500)```
