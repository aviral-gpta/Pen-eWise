tags=[]
pos=4

def checkTags(tags):
    prevTag=tags[0]
    last=True
    count=0
    for idx in range(1,len(tags)):
        if((prevTag>>3==1 and tags[idx]>>3==2)or(prevTag>>3==2 and tags[idx]>>3==1)):
            if(last):
                if(((prevTag>>1)&0b11)==((tags[idx]>>1)&0b11)):
                    last=False
                    prevTag=tags[idx]
                    continue
                else:
                    count+=1
            else:
                if((((prevTag>>1)&(0b11))+1)%4==((tags[idx]>>1)&(0b11))):
                    prevTag = tags[idx]
                    last=True
                    continue
                else:
                    count+=1
        else:
            count+=1
    return count


with open('sensor_data_50.txt', 'r') as file:
    for line in file:
        words=line.split()
        tags.append(int(float(words[pos])))
cnt=checkTags(tags)
print("Number of incorrect tags: ",cnt)
if checkTags(tags)==0:
    print("Tags are correct")