#!/bin/bash
# Usage parse_caffe_log.sh /path/to/caffe.INFO
# It creates the following two text files, each containing a table:
#     caffe_info_train.txt (columns: 'iter acc loss lr')
#     caffe_info_val.txt (columns: 'iter acc loss')

if [ "$#" -lt 1 ]
then
echo "Usage parse_caffe_log.sh /path/to/caffe.INFO"
exit
fi
LOG=`basename $1`
sed -n '/Iteration .* Testing net/,/Iteration *. loss/p' $1 > aux.txt
sed -i '/Waiting for data/d' aux.txt
sed -i '/prefetch queue empty/d' aux.txt
sed -i '/Iteration .* loss/d' aux.txt
sed -i '/Iteration .* lr/d' aux.txt
sed -i '/Train net/d' aux.txt

echo '#iter' > aux0.txt
echo 'acc' > aux1.txt
echo 'loss' > aux2.txt

grep 'Iteration ' aux.txt | sed  's/.*Iteration \([[:digit:]]*\).*/\1/g' >> aux0.txt
grep 'Test net output #0' aux.txt | awk '{print $11}' >> aux1.txt
grep 'Test net output #1' aux.txt | awk '{print $11}' >> aux2.txt

# Generating
paste aux0.txt aux1.txt aux2.txt | column -t >> caffeINFOtrain.txt
rm aux.txt aux0.txt aux1.txt aux2.txt

grep '] Solving ' $1 > aux.txt
grep ', loss = ' $1 >> aux.txt

echo '#iter' > aux0.txt
echo 'acc' > aux4.txt
echo 'loss' > aux1.txt
echo 'lr' > aux2.txt

grep 'Iteration ' aux.txt | sed  's/.*Iteration \([[:digit:]]*\).*/\1/g' >> aux0.txt
grep ', lr = ' $1 | awk '{print $9}' >> aux2.txt
grep 'Train net output #0' $1 | awk '{print $11}' >> aux4.txt
grep 'Train net output #1' $1 | awk '{print $11}' >> aux1.txt

# Generating
paste aux0.txt aux4.txt aux1.txt aux2.txt | column -t >> caffeINFOval.txt
rm aux.txt aux0.txt aux1.txt aux2.txt aux4.txt





