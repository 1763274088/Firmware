
if [  $1!='' ]
then
   msg=$1;
else
   msg='push�ļ������Ź۲��������������';
fi

git status

git add .

git commit -m '$msg'

git push

git status