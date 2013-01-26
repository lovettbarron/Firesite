#include "testApp.h"

using namespace cv;
using namespace ofxCv;

//--------------------------------------------------------------
void testApp::setup(){
    ofSetVerticalSync(true);
//    ofEnableLighting();
 
    box2d.init();
	box2d.setGravity(0, 0);
    box2d.createBounds(0,0,2000,400);
	box2d.setFPS(10.0);
	box2d.registerGrabbing();
    
    for (int i=0; i<5; i++) {
		
        ofxBox2dCircle home;
        home.setup(box2d.getWorld(), i*128,100,40);

		ofxBox2dCircle circle;
		circle.setPhysics(9999.0, 0.01 , 2.9);
		circle.setup(box2d.getWorld(),  i*128,100, 4);
		boxes.push_back(circle);
	}
    
//    testLight = new Light(ofVec3f(0,0,0), 0);
    
    camera.disableMouseInput();
    
    numberOfLights = 5;
    room = ofVec3f(1000,400,500);
    
    for(int i=0;i<numberOfLights;i++) {
        lights.push_back( new Light(ofVec3f(ofRandom(0,room.x), room.y, ofRandom(0,room.z) ), i, 10+i, 12+i, &serial) );
    }
    
//    for(int i=0;i<10;i++) {
//        people.push_back( new People( ofVec3f(ofRandom(0,room.x),0,ofRandom(0,room.z)),i));
//    }
    setupGUI();   
    setupCamera();
    for(int i=0;i<1;i++) {
        cameras.push_back( new Camera( ofVec3f(), 0, &kinect) );
    }
    background.setLearningTime(900);
	background.setThresholdValue(10);
    setupArduino();
}

void testApp::setupArduino() {

    
//    New proto
    // Lights in ID order, CSV new line delin
 
    serial.listDevices();
    //	vector <ofSerialDeviceInfo> deviceList = serial.getDeviceList();
	serial.setup(9,9600); 
    //    int numSent = serial.writeBytes("Setup Firesite");
    //ofLog() << "Sent " << ofToString(numSent) << " bytes.";
    
}

void testApp::writeArduino() {
    if(limitBuffer <= ofGetElapsedTimeMillis()) {
        limitBuffer = ofGetElapsedTimeMillis() + panel.getValueI("TimeBetweenUpdate");
        buffer = "";
        for(int l = 0; l<lights.size();l++) {
            lights[l]->update();

            serial.writeByte((unsigned char)floor(lights[l]->getStrength() * 127));

            buffer += ofToString((int)(lights[l]->getStrength() * 127));

            if(l!=lights.size()-1) buffer += ",";

            serial.writeByte(',');
        }
        buffer += "\n";
        serial.writeByte('\n');
//        serial.writeBytes((unsigned char*) buffer.c_str(),buffer.size());
        ofLog() << buffer;
    }
}

void testApp::exit() {
    serial.writeByte('e');
    kinect.close();
    kinect.clear();
}

void testApp::setupGUI() {
    panelWidth = 300;
    panel.setup(panelWidth, 1024);
    
    panel.addPanel("PointCloud");
    panel.addSlider("cameraDistance",700,0,1000,false);
    panel.addSlider("TimeBetweenUpdate", 1000, 0,5000,true);
    panel.addLabel("Camera");
    panel.addSlider("cam1x", 0, 0., 1., false);
    panel.addSlider("cam1y", 0, 0., 1., false);
    panel.addSlider("cam1z", 0, 0., 1., false);
    panel.addSlider("cam1d", 0, 0., 1., false);
    
    for( int i=0;i<numberOfLights;i++) {
        panel.addPanel("Light" + ofToString(i));    void setLocation(ofVec3f _position);
        panel.addLabel("Light" + ofToString(i) );
        panel.addSlider("l" + ofToString(i) + "pwr", 1., 0., 1., false);
        panel.addSlider("l" + ofToString(i) + "x", lights[i]->getLocation().x / room.x, 0., 1., false);
        panel.addSlider("l" + ofToString(i) + "y", lights[i]->getLocation().y / room.y, 0., 1., false);
        panel.addSlider("l" + ofToString(i) + "z", lights[i]->getLocation().z / room.z, 0., 1., false);
    }
    
    panel.addPanel("Tracking Bits");
    panel.addLabel("Image Processing");
    panel.addSlider("cvBlur",10,0,100,true);
    panel.addSlider("maxThreshold", 15, 0, 255, true);
    panel.addSlider("minAreaRadius", 7, 0, 640, true);
    panel.addSlider("maxAreaRadius", 100, 0, 640, true);
    panel.addSlider("DepthMultiplier", .01,0,5.,false);
    panel.addLabel("Background Subtraction");
    panel.addSlider("learningTime",900,0,2000,true);
    panel.addSlider("backgroundThresh",10,0,50,true);
    panel.addToggle("resetBg", false);
    panel.addSlider("OverlapDistance", 500, 0,1000,true);
    
    panel.addSlider("idScale",.3,0,1.,false);
    panel.addSlider("idPosx",-500,-700,700,true);
    panel.addSlider("idPosy",-500,-700,700,true);
    
    panel.addPanel("Kinect");   
    panel.addSlider("angle", 0, -40, 40, true);
    angle = panel.getValueI("angle");
}

void testApp::setupCamera() {
    kinect.setRegistration(true);
    ofLog() << "Starting first kinect";
    kinect.init(false, false, true); // infrared=false, video=true, texture=true
    kinect.open(0);
    kinect.setCameraTiltAngle(angle);
    
    if(!kinect.isConnected()) {
        cam.initGrabber(640, 480);
    }
    
    thresh.allocate(640, 480, OF_IMAGE_GRAYSCALE);
    kDepth.allocate(640, 480, OF_IMAGE_GRAYSCALE);
    kDepthMat.create(480, 640, CV_8UC1);
    //threshMat.create(480, 640, CV_32F);
    imitate(threshMat, kDepthMat);
    imitate(avgMat, kDepthMat);
    //    imitate(kDepth, kDepthMat);
    //    imitate(thresh, threshMat);
    
    background.setThresholdValue(panel.getValueI("backgroundThresh"));
    background.setLearningTime(panel.getValueI("learningTime"));
    
    contourFinder.setMinAreaRadius(panel.getValueI("minAreaRadius"));
    contourFinder.setMaxAreaRadius(panel.getValueI("maxAreaRadius"));
    contourFinder.setThreshold(panel.getValueI("maxThreshold"));
    // wait for half a frame before forgetting something
    contourFinder.getTracker().setPersistence(15);
    // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(32);
    avgCounter = 0;
}

void testApp::updateCamera() {
    contourFinder.setMinAreaRadius(panel.getValueI("minAreaRadius"));
    contourFinder.setMaxAreaRadius(panel.getValueI("maxAreaRadius"));
//    contourFinder.setThreshold(panel.getValueI("maxThreshold"));
    contourFinder.setAutoThreshold(true);
    contourFinder.setInvert(true);
    
    background.setLearningTime(panel.getValueI("learningTime"));
    background.setThresholdValue(panel.getValueI("backgroundThresh"));
    
    if(panel.hasValueChanged("angle")) {
        angle = panel.getValueI("angle");
        kinect.setCameraTiltAngle(angle);
    }   
    
    
    if(panel.getValueB("resetBg")) {
        //        background.reset();
        threshMat = kDepthMat.clone();
        panel.setValueB("resetBg",false);
    }
    
    if(!kinect.isConnected()) {
    } else {
        kinect.update();
       if(kinect.isFrameNew()) {
           if(avgCounter == 0) {
               kDepthMat = toCv(kinect.getDepthPixelsRef());
               blur(kDepthMat, panel.getValueI("cvBlur"));
               avgMat = kDepthMat;
               avgCounter++;
           } else
           if(avgCounter < 2) { // If averaging the frames
               kDepthMat = toCv(kinect.getDepthPixelsRef());
               blur(kDepthMat, panel.getValueI("cvBlur"));
               avgMat += kDepthMat/3;
               avgCounter++;
               
           } else { // Act on the average
               cameras[0]->isNewFrame(true);   
               kDepthMat = toCv(kinect.getDepthPixelsRef());
               blur(kDepthMat, panel.getValueI("cvBlur"));
               avgMat += kDepthMat/3;
               // threshMat = ( (kDepthMat * .3) + (threshMat))/2 ; // Attempt at an adapting threshold...
               cv::absdiff(avgMat, threshMat, avgMat);
               fillHoles(avgMat);
               contourFinder.findContours(avgMat);
               toOf(avgMat,kDepth);
               kDepth.update();
              // brush = getContour(&contourFinder);
                 float distance;
               for(int j=0;j<lights.size();j++) {
                   distance = 0;
                   for( int i=0;i<contourFinder.size();i++) {
                       ofPoint center = toOf(contourFinder.getCenter(i));
                       distance += lights[j]->getLocation().squareDistance(ofVec3f(center.x,30,center.y)) * .001;
                   }
                   lights[j]->setTotalDist(distance);
               }
               
               // This is the weird bit that chooses which light is active
               int pwrLight = 0;
               for(int i=0;i<lights.size();i++) {
                   lights[i]->isActive(false);
                   if(lights[i]->getTotalDist() < lights[pwrLight]->getTotalDist()) { 
                       pwrLight = i; 
                   }
               }
               lights[pwrLight]->isActive(true); 
               avgCounter = 0;
          }
       } else {
           cameras[0]->isNewFrame(false);
       }
       
     
    }
}


// This was taken from
// http://www.morethantechnical.com/2011/
// 03/05/neat-opencv-smoothing-trick-when-kineacking-kinect-hacking-w-code/
void testApp::fillHoles(cv::Mat src, cv::Mat dst) {
    cv::Mat depthf(cv::Size(640,480),CV_8UC1);
    src.convertTo(depthf, CV_8UC1, 255.0/2048.0);
    cv::Mat _tmp,_tmp1; //minimum observed value is ~440. so shift a bit
    cv::Mat(depthf - 400.0).convertTo(_tmp1,CV_64FC1);
    
    ofPoint minLoc; double minval,maxval;
    minMaxLoc(_tmp1, &minval, &maxval, NULL, NULL);
    _tmp1.convertTo(depthf, CV_8UC1, 255.0/maxval);  //linear interpolation
    
    //use a smaller version of the image
    cv::Mat small_depthf; cv::resize(depthf,small_depthf,cv::Size(),0.2,0.2);
    //inpaint only the "unknown" pixels
    cv::inpaint(small_depthf,(small_depthf == 255),_tmp1,5.0,INPAINT_TELEA);
    
    resize(_tmp1, _tmp, depthf.size());
    imitate(dst,depthf);
    _tmp.copyTo(dst, (dst == 255));
 //   ofxCv::copy(depthf,dst);
}

void testApp::fillHoles(cv::Mat _mat) {
    fillHoles(_mat, _mat);
}


//--------------------------------------------------------------
void testApp::update(){
	box2d.update();	
    updateCamera();
    depthMulti = panel.getValueF("DepthMultiplier");
	ofVec2f mouse(ofGetMouseX(), 5);
	float minDis = ofGetMousePressed() ? 300 : 200;
    
    camera.setTarget(ofVec3f(room.x/2,room.y/2,room.z/2));
    camera.setPosition(room.x, room.y, room.z);
    camera.setDistance(panel.getValueF("cameraDistance"));
//    for(int i=0;i<lights.size();i++) {
//    }
    
    for(int i=0;i<cameras.size();i++) {
        ofVec3f camPosition(
            panel.getValueF("cam1x") * room.x,
            panel.getValueF("cam1y") * room.y,
            panel.getValueF("cam1z") * room.z
        );
        cameras[i]->setLocation(camPosition);
        cameras[i]->setDirection(panel.getValueF("cam1d"));
        cameras[i]->update();
    }
    
//    for(int i=0;i<people.size();i++) {
//        people[i]->update();
//    }

    updateSettings();
    writeArduino();
}

//--------------------------------------------------------------
void testApp::updateSettings(){
    for( int i=0;i<numberOfLights;i++) {
        float pwr = panel.getValueF("l" + ofToString(i) + "pwr");
        float x = panel.getValueF("l" + ofToString(i) + "x");
        float y = panel.getValueF("l" + ofToString(i) + "y");
        float z = panel.getValueF("l" + ofToString(i) + "z");
        lights[i]->setLocation(ofVec3f(x*room.x,y*room.y,z*room.z));
        if(sliderControl) lights[i]->setStrength(pwr);
    }
}

//--------------------------------------------------------------
void testApp::draw(){
    ofBackgroundGradient(ofColor(200),ofColor(170) );    
    
    glEnable(GL_DEPTH_TEST);
    
	camera.begin();	
//        ofTranslate(0,-400);
        if(rotate)
            ofRotateY(ofRadToDeg(  ofGetElapsedTimeMillis()*.0001 ));
    
        ofSetColor(200);
        
//        testLight->draw();
        for(int i=0;i<lights.size();i++) {
            lights[i]->draw();
        }
//        for(int i=0;i<people.size();i++) {
//            people[i]->draw();
//        }
    
        for(int i=0;i<cameras.size();i++) {
            cameras[i]->draw();
        }
    
        for(int i = 0; i < contourFinder.size(); i++) {
            ofPoint center = toOf(contourFinder.getCenter(i));
            //ofSphere(center.x, 10, center.y, 20);
            float depth, multi;
            if(kinect.isConnected()) {
                depth = kinect.getDistanceAt(center)*.1;
                multi = 1 * pow((float)depth,1.1f);
            } else {
                multi = 1 * pow((float)center.y,1.1f);
            }
            drawPerson(ofPoint(multi,  center.x), toOf(contourFinder.getVelocity(i)));
        }
    
        ofPushMatrix();
            ofSetColor(100,100,100);
            ofTranslate(room.x/2,0,room.z/2);
            ofScale(room.x,0,room.z);
            ofBox(1);
        ofPopMatrix();
    
    camera.end();
    
    
    
    if(debug) {
    ofSetHexColor(0x123456);
    
        for(int i=0;i<lights.size();i++) {
            lights[i]->debug(); 
        }
    
    for(int i = 0; i < contourFinder.size(); i++) {
        ofPoint center = toOf(contourFinder.getCenter(i));
        float depth;
        if(kinect.isConnected())
            depth = kinect.getDistanceAt(center)*.1;
        else depth = 1.0;
        
//        ofEllipse((center.x/640) * ofGetWidth(),100, depth,depth);
    }
    drawCamDebug();
    }
    

    
    glDisable(GL_ALPHA_TEST);
    glDisable(GL_DEPTH_TEST);
}

void testApp::drawPerson(ofPoint pos, ofVec3f dir) {
    ofPushMatrix();
    ofTranslate(pos.x,30,pos.y);
    ofSetColor(255);
    ofDrawBitmapString("x" + ofToString(pos.x) + " y" + ofToString(pos.y),0,0);
    ofSetColor(127);
    ofSphere(0,60,0,20);
    ofScale(20,60,10);
    ofBox(1);
    ofLine(0,0,dir.x,dir.y);
    ofPopMatrix();
}

void testApp::drawCamDebug() {
    
    ofPushMatrix();
    glDisable(GL_DEPTH_TEST);
    ofTranslate(ofGetWidth()-(kinect.getWidth()/2), ofGetHeight()-(kinect.getHeight()/2));
    ofScale(.5,.5);
    ofSetColor(255);
    ofSetLineWidth(1);
        if(!kinect.isConnected()) cam.draw(0, 0);
        else kDepth.draw(0,0);
        
//        ofImage debug = toOf(kDepthMat);
        contourFinder.draw();
        
        for(int i = 0; i < contourFinder.size(); i++) {
            ofPoint center = toOf(contourFinder.getCenter(i));
            
            float depth;
            if(kinect.isConnected())
                depth = kinect.getDistanceAt(center);
            else depth = 0;;
            ofSetColor(255);
            contourFinder.getPolyline(i).draw();
            ofPushMatrix();
            ofTranslate(center.x, center.y);
            int label = contourFinder.getLabel(i);
            ofDrawBitmapString(ofToString(label) + ":" + ofToString(depth*depthMulti), 0, 12);
            ofVec2f velocity = toOf(contourFinder.getVelocity(i));
            ofScale(5, 5);
            ofLine(0, 0, velocity.x, velocity.y);
            ofPopMatrix();
    }
    glDisable(GL_DEPTH_TEST);
//    thresh.draw(thresh.width, 0, 2, 256,192);
    ofPopMatrix(); // For panel
}


//--------------------------------------------------------------
void testApp::keyPressed(int key){
    switch(key) {
        case ' ':
            break;
        case 'i':
            sliderControl = !sliderControl;
            for(int i=0;i<lights.size();i++) {
                lights[i]->isDebug(sliderControl);
            }
            break;
        case 'r':
            rotate =!rotate;
        case 'd':
            debug = !debug;
            break;
            
        case 'm':
			if(camera.getMouseInputEnabled()) camera.disableMouseInput();
			else camera.enableMouseInput();
			break;
        case 'b':
            ofxCv::copy(kDepthMat, threshMat);
            break;
    }
}


//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------

Light::Light(ofVec3f _position, int _id, int _clock, int _data, ofSerial * _arduino) {
    position = _position;
    lightId = _id;
    clockPin = _clock;
    dataPin = _data;
    ledCount = 20;
    ledPerMeter = 48;
    width = 10;
    height = 100;
    numOfArms = 3;
    power = 0.;
    active = false;
    tweenTime = 2000;
    changed = 0;
    arduino = _arduino;
    
    for(int i=0;i<numOfArms;i++) {
//        arduino->sendDigitalPinMode(clockPin+i, ARD_PWM);
  //      arduino->sendDigitalPinMode(dataPin+i, ARD_PWM);
    }
    
    for(int i=0;i<ledCount*numOfArms;i++) {
        leds.push_back(ofNoise(i,i,i));
    }
    
}
    
Light::~Light() {
    
}

void Light::isDebug(bool _debug) {
    lightDebug = _debug;
}

void Light::isActive(bool _active) {
    active = _active;
    if( power <  .5) 
        changed = ofGetElapsedTimeMillis() + tweenTime;
}

void Light::setStrength(float _power) {
    power = _power;
}

void Light::draw() {
    ofPushMatrix();
    ofTranslate(position);
    ofSetColor(255);
    ofDrawBitmapString("Light" + ofToString(lightId),0,12);
    ofDrawBitmapString("x" + ofToString(position.x) + " y" + ofToString(position.z),0,0);
    ofRotate(110,-1,0,0);
    for(int i=0;i<numOfArms;i++) {
        int rotation = i * (360/numOfArms);
        ofPushMatrix();
            ofRotate(rotation,0,-1,1);
            ofRotate(45,0,-1,0);
            drawArm(i);
        ofPopMatrix();
    }
    ofPopMatrix();
}

void Light::tweenUpdate() {
    if(!lightDebug) {
        if(active) {
            if(power < 1.0)
                power = (ofGetElapsedTimeMillis() - changed) / tweenTime;
            else power = 1.0;
        } else {
            if(power > 0.) power -= .1;
            else power = 0;
        }
    }
    power = ofClamp(power,0.,1.);
}

void Light::lightUpdate() {
    float maxPower, minPower, ptr, ledStr;
    
//    int per = (leds.size() / ledCount) * power;
  //  int per = floor(( ledCount * (1-power)) + 1);
    //ofLog() << ofToString(per);
//    for(int i=0;i<leds.size();i+=ofRandom(per*.5,per)) {
//        leds[i] = ofRandom(0,1);//ofNoise(leds[i]);
//    }
//    if(!per) per = leds.size() / numOfArms;
/*    for(int i=0;i<leds.size();i++) {
        float maxPower = 1- ( (i % ledCount) / ledCount);
        if(i%per == 0) leds[i] = ofRandom(0,maxPower);
        else if (ofRandom(0,4) == 0) leds[i] = ofRandom(0,1);
        else leds[i] = 0;
    } */
    for( int a=0;a<numOfArms;a++) { // This code is replicated on the arduino.
        for(int l=0;l<ledCount;l++) {
            maxPower = (float)l / (float)ledCount;
            minPower = 1.0f - (maxPower * (1.0f-power));
           // float ledStr = abs(minPower - power);
            ptr = l + ( a * ledCount );
            leds[ptr] = ofRandom(minPower,maxPower) * power;
            //ofLog() << ofToString(leds[ptr]);
        }
    }
    
}

void Light::update() {
    tweenUpdate();
    lightUpdate();
}

unsigned char * Light::getBuffer() {
    unsigned char * bufferPtr;
    bufferPtr = &buffer[0];
//    ofLog() << ofToString(bufferPtr);
    string buff;
    for(int i=0;i<buffer.size();i++) {
        buff += buffer[i];
    }
    ofLog() << buff;
    return bufferPtr;
}

int Light::getBufferLength() {
   return buffer.size();
}

void Light::setLocation(ofVec3f _position) {
    position = _position;
}

ofVec3f Light::getLocation() {
    return position;
}

void Light::drawArm(int num) {
    ofSetColor(255);
    ofPushMatrix();
        ofTranslate(width/2,height/2,width/2);
        ofSetColor(56,19,11);
        ofPushMatrix();
            ofScale(width,height, width);
            ofBox(1);
        ofPopMatrix();
    ofPopMatrix();
    for(int i=0;i<ledCount;i++) {
        ofPushMatrix();
        ofTranslate(0, i * (height/ledCount), width*.1 );
        ofSetColor(leds[i+(ledCount*num)]*255, (leds[i+(ledCount*num)]) * 127, 0);
//        ofDrawBitmapString(ofToString(i+(ledCount*num)) + ":" + ofToString(leds[i+(ledCount*num)]),0,10);
        ofSphere(width/2);
        ofPopMatrix();
    }
}

void Light::setTotalDist(float _dist) {
    totalDist = _dist;
};

float Light::getTotalDist() {
    return totalDist;
};

float Light::getStrength() {
    return power;
}

void Light::debug() {
    glDisable(GL_DEPTH_TEST);
    ofPushMatrix();
    ofTranslate(ofGetWidth()-(120),(lightId*120)+40);
    int arm = 0;
    ofSetColor(255);
    ofDrawBitmapString("Light" + ofToString(lightId),0,0);
    ofDrawBitmapString("Dist" + ofToString(totalDist),0,12);
    ofDrawBitmapString("Power" + ofToString(power),0,24);
    for(int i=0;i<leds.size();i++) {
        if(i%ledCount == 0) {
            arm+=1;
            ofPushMatrix();
            ofTranslate(-50,arm*20);
            ofSetColor(255);
            ofDrawBitmapString("arm" + ofToString(arm),0,12);
            ofPopMatrix();
        }
        ofPushMatrix();
            ofScale(5,1);
            ofTranslate(i%ledCount,arm*20);
            ofSetColor(leds[i]*255, 100, 100);
            ofSetLineWidth(3);
            ofLine(0,(1-leds[i])*20,0,20);
            ofSetLineWidth(1);
        ofPopMatrix();
    }
    ofPopMatrix();
    glEnable(GL_DEPTH_TEST);
}

float Light::lin2log(float _lin) {
    return log10(_lin);
}


//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------

People::People(ofVec3f _position, int _id) {
    position = _position;
    personId = _id;
    width = 10;
    height = 100;
    
}

People::~People() {
    
}

void People::update() {
    position += ofVec3f(ofRandom(-10,10),0,ofRandom(-10,10));
}

void People::draw() {
    ofPushMatrix();
    ofTranslate(position);
    ofScale(width,height,width);
    ofSetColor(127,50,50);
    ofSphere(1);
    ofPopMatrix();
}



//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------
//--------------------------------------------------------------


Camera::Camera(ofVec3f _position, int _id, ofxKinect * _kinect) {
    position = _position;
    personId = _id;
    kinect = _kinect;
    width = 100;
    height = 10;
    depth = 10;
    res = 20;
    direction = 0;
    bFrame = false;
}

Camera::~Camera() {
    
}

void Camera::update() {

}

void Camera::isNewFrame(bool _kFrame) {
    bFrame = _kFrame;
}

void Camera::draw() {
    ofPushMatrix();
    ofTranslate(position);
    ofRotateY(direction*360);
    ofRotateX(kinect->getCurrentCameraTiltAngle());
     if(bFrame) { 
        ofSetColor(255,0,0);
        ofSphere(5);
//           for(int y=0;y<kinect->getHeight()-res;y+=res) {
//               for(int x=0;y<kinect->getWidth()-res;x+=res) {
////                   int ptr = x + (y * kinect->getWidth());
//                  // ofRect(kinect->getWorldCoordinateAt(x,y), 10,10);
//               }
//           }
        }
    ofScale(width,height,depth);
    ofSetColor(0);
    ofBox(1); 
    ofPopMatrix();
}

void Camera::setLocation(ofVec3f _position) {
    position = _position;
}

void Camera::setDirection(float _direction){
    direction = _direction;
}

ofVec3f Camera::getLocation() {
    return position;
}