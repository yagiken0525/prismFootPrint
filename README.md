# Team Prism 足あと検出

### 今後やること
- Structure from Motion で部屋の3次元点取得 OK
- 平面フィッティングで床の3次元位置取得 OK
- 前の足あと検出アルゴリズムで足あと計測
- openPoseの関節位置を3次元復元

- キャリブレーション
- 実行時間短縮
    1. 点群再投影
    2. Searching rectの大きさに画像を分割
    3. 再投影点群を枠ごとに分割保持
    
##### 11/25
- TODO
    - homography 変換
    - HeatMap 完成
    - 長時間撮影
    - 行動認識 