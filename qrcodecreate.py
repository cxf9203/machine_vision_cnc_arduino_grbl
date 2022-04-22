import qrcode

data= "wo shi tiancai"  #数据内容
img_file="ceshi.jpg" #保存的图片
img=qrcode.make(data)#内容进二维码
img.save(img_file)#保存图片
img.show()#展示图片
