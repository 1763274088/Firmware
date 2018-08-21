
if [  $1!='' ]
then
   msg=$1;
else
   msg='push文件、干扰观测器函数编译测试';
fi

git status

git add .

git commit -m '$msg'

git push

git status