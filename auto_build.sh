#!/bin/bash

# --- 설정 변수 ---
# 1. 빌드할 파일 이름 (ECC 포함된 2MB 파일)
TARGET_ROM="tcc70xx_pflash_boot_2M_ECC.rom"

# 2. 경로 설정
BASE_DIR=~/topst-vcp
BUILD_DIR=$BASE_DIR/build/tcc70xx/gcc
TOOLS_DIR=$BASE_DIR/tools/fwdn_vcp
WINDOWS_DIR=/mnt/c/topst_fw

echo "========================================"
echo ">> 1. Clean & Build Start..."
echo "========================================"

# 3. 빌드 폴더로 이동해서 싹 지우고 다시 빌드 (Make Clean & Make)
cd $BUILD_DIR
make clean
make

# 빌드 성공 확인 (파일이 없으면 중단)
if [ ! -f "output/$TARGET_ROM" ]; then
    echo "!!!!!! BUILD FAILED (File not found) !!!!!!"
    exit 1
fi

echo "========================================"
echo ">> 2. Copying ROM to tools directory..."
echo "========================================"

# 4. 빌드된 파일을 툴 폴더로 복사 (이름 절대 바꾸지 않음!)
cp output/$TARGET_ROM $TOOLS_DIR/

echo "========================================"
echo ">> 3. Deploying to Windows ($WINDOWS_DIR)..."
echo "========================================"

# 5. 윈도우 폴더 초기화 및 복사
# (윈도우 폴더를 지우고 새로 만들어서 항상 깨끗하게 유지)
rm -rf $WINDOWS_DIR
mkdir -p $WINDOWS_DIR
cp -r $TOOLS_DIR/* $WINDOWS_DIR/

echo ">> SUCCESS! Windows C:\topst_fw 폴더로 파일 복사 완료!"
echo ">> 이제 윈도우에서 fwdn_vcp.bat를 실행하세요."