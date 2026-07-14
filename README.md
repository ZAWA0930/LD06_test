# LD06_test
nkym_hrkさんへ

:: プロジェクトフォルダ作成
mkdir LD06
cd LD06

:: 仮想環境作成
python -m venv venv

:: 仮想環境有効化
venv\Scripts\activate

:: pip最新版へ更新（推奨）
python -m pip install --upgrade pip

:: 必要ライブラリをインストール
pip install pyserial numpy opencv-python

:: インストール確認
pip list

:: Pythonのバージョン確認
python --version

:: プログラム実行
python LD06_test.py


:: 仮想環境終了
deactivate
