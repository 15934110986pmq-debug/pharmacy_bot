python3 fk_setup.py build_ext --inplace
python3 ik_setup.py build_ext --inplace
rm ./*.c
rm ./build/ -rf
mv forward_kinematics.*.so forward_kinematics.so
#mv inverse_kinematics.*.so inverse_kinematics.so
