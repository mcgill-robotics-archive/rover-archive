#JAN 1ST 2:30PM -- IT WORKS!!!!!
#MONTREAL IS KICKING BOSTONS' ASS AT THE WINTER CLASSIC
#2016 FUCK YEAH!
import cv2
import numpy as np
import time

vals = []
setCount = 0
setMode = True

#test = True

# build the mapping
def buildMap(Ws,Hs,Wd,Hd,R1,R2,Cx,Cy):
    map_x = np.zeros((Hd,Wd),np.float32)
    map_y = np.zeros((Hd,Wd),np.float32)
    for y in range(0,int(Hd-1)):
        for x in range(0,int(Wd-1)):
            r = (float(y)/float(Hd))*(R2-R1)+R1
            theta = (float(x)/float(Wd))*2.0*np.pi
            xS = Cx+r*np.sin(theta)
            yS = Cy+r*np.cos(theta)
            map_x.itemset((y,x),int(xS))
            map_y.itemset((y,x),int(yS))
        
    return map_x, map_y
# do the unwarping 
def unwarp(img,xmap,ymap):
    output = cv2.remap(img,xmap,ymap,cv2.INTER_LINEAR)
    return output

#mouse callback function
def setCoord(event,x,y,flags,param):
    global vals, setCount
    if event == cv2.EVENT_LBUTTONDBLCLK:
        coord = (x,y)
        vals.append(coord)
        setCount = setCount+1
        print vals
        print vals[0][1]
        print setCount


cap = cv2.VideoCapture(0)

#If remains too long in this mode, bug
while (setMode):
    
    ret, frame = cap.read()
    
    cv2.imshow('SetupFeed', frame)
    cv2.setMouseCallback('SetupFeed',setCoord)

    if setCount == 3:
        setMode = False

    if cv2.waitKey(1) == ord('q'):
        break

# 0 = xc yc
# 1 = r1
# 2 = r2
# center of the "donut"    
Cx = vals[0][0]
Cy = vals[0][1]
# Inner donut radius
R1x = vals[1][0]
R1y = vals[1][1]
R1 = R1x-Cx
# outer donut radius
R2x = vals[2][0]
R2y = vals[2][1]
R2 = R2x-Cx
# our input and output image siZes
Wd = 2.0*((R2+R1)/2)*np.pi
Hd = (R2-R1)
Ws = frame.shape[0]
Hs = frame.shape[1]
# build the pixel map, this could be sped up
print "BUILDING MAP!"
xmap,ymap = buildMap(Ws,Hs,Wd,Hd,R1,R2,Cx,Cy)
print "MAP DONE!"
# do an unwarping and show it to us

while(True):
    
    ret, frame = cap.read()
      
    result = unwarp(frame,xmap,ymap)
    
    #need to convert to an numpy array to show ?
    cv2.imshow('test',result)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

"""
# SimpleCV/OpenCV video out was giving problems
# decided to output frames and convert using
# avconv / ffmpeg. 

# I used these params for converting the raw frames to video
# avconv -f image2 -r 30 -v:b 1024K -i samples/lapinsnipermin/%03d.jpeg output.mpeg
i = 0
while img is not None:
    result = unwarp(img,xmap,ymap)
    # Once we get an image overlay it on the source
    derp = img.blit(result,(0,img.height-result.height))
    derp = derp.applyLayers()
    derp.save(disp)
    fname = "FRAME{num:05d}.png".format(num=i)
    derp.save(fname)
    #vs.writeFrame(derp)
    # get the next frame
    img = vc.getImage()
    i = i + 1
"""
