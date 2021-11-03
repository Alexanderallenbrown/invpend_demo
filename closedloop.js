

//timing globals
var t;
var dt;
var oldt;
//touchscreen globals
var Im_touched = false;
//slider(s)
var goalslider = (document.getElementById("goalslider"));
var checkBox = document.getElementById("controlActive");
var t2goal = 0;
var kp = 0.05;
var kd = 0.01;
var kicktorque = 0;

//set up callbacks for the HTML sliders.
goalslider.oninput = function(){
  t2goal = this.value/10000.0;
}
kpslider.oninput = function(){
  kp = this.value/1000.0;
}
kdslider.oninput = function(){
  kd = this.value/10000.0;
}

//button callback
function kickCallback(){
  kicktorque = 1;
}


//physical variables
var l1 = .2;
var l2 = .3;
var m1 = .1;
var m2 = .1;
var J1 = 1./12*m1*l1*l1;
var J2 = 1./12*m2*l2*l2;
var b1 = .05;
var b2 = .003;





function setup() {
  t = millis();
  oldt = t;
  var cnv = createCanvas(500, 500,WEBGL);
  cnv.parent('sketch-holder');

  //create objects
  //for rods, 1/3ml^2 is good.

  //Pend(scale,l1,l2,m1,m2,J1,J2,b1,b2)
  pend = new RPend(300,l1,l2,m1,m2,J1,J2,b1,b2);
  pend.x[2]=0.01
  pend.x[0] = PI

}


function draw() {
  
  //set background
  background(200);
  stroke(0);
  fill(255);
  //take care of timing
  t = millis() / 1000.0;
  dt = (t - oldt);
  oldt = t;

  //normalMaterial();
  //orbitControl();
  //get into the correct view angle
  rotateX(-0.2*PI);
  
  
  var controloff = checkBox.checked;
  //update objetcts
  if(!controloff){
  var mytau = (-t2goal+pend.x[2])*kp+pend.x[3]*kd
  }
else{
  var mytau=0;
}
  // var mytau = (-t2goal+pend.x[2])*kp+pend.x[3]*kd;
  if(millis()>2000){
  pend.update(dt,mytau);
}
  if (kicktorque>0){
    kicktorque=0;//one shot on the kick torque
  }
  // pend.update(dt,tau)
  //get out of view angle
  rotateX(0.2*PI)

  console.log(dt)
  
}


function RPend(scale,l1,l2,m1,m2,J1,J2,b1,b2){
  this.l1 = l1;
  this.l2 = l2;
  this.m1 = m1;
  this.m2 = m2;
  this.J1 = J1;
  this.J2 = J2;
  this.b1 = b1;
  this.b2 = b2;
  this.x = [0,0,0,0]// order is t1,t1d,t2,t2d
  this.g = 9.81

  this.display = new RPend_Display(0,0,50,50,.05*scale*l1,scale*l2,.05*scale*l1,scale*l2);

  this.update = function(dt,tau){
    this.updateStates(dt,tau);
    // console.log(this.x)
    this.display.draw(this.x[0],this.x[2])//draw based on theta1, theta2
  }

this.updateStates = function(dt,tau){
  //RK step 1
  var k1x = this.stateDerivs(tau,this.x);
  var xhat1 = this.vectoradd(this.x, this.scalarmult(dt,k1x))
  // console.log(xhat1)
  //RK step 2
  var k2x = this.stateDerivs(tau,xhat1)
  var xhat2 = this.vectoradd(this.x, this.scalarmult( dt/2,k2x))
  //RK step 3
  var k3x = this.stateDerivs(tau,xhat2)
  var xhat3 = this.vectoradd(this.x,this.scalarmult(dt,k3x))
  //RK step 4
  var k4x = this.stateDerivs(tau,xhat3)

  var sum1 = this.vectoradd(k4x,this.scalarmult(2,k3x))
  var sum2 = this.vectoradd(sum1,this.scalarmult(2,k2x))
  var sum3 = this.vectoradd(sum2,k1x)
  var xdot = this.scalarmult(1.0/6,sum3)
  //do the final update.
  this.x = this.vectoradd(this.x,this.scalarmult(dt,xdot))
  // this.x = xhat1
}

this.scalarmult = function(scalar,vector){
  var res = vector;
  for(i=0;i<vector.length;i++){
    res[i] = vector[i]*scalar;
  }
  return res
}

this.vectoradd = function(v1,v2){
  var res = v1;
  for(i=0;i<v1.length;i++){
    res[i] = v1[i]+v2[i]
  }
  return res
}

this.stateDerivs = function(tau,x){
  //first pull out states to make local variables
  var t1 = x[0]
  var t2 = x[2]
  var t1d = x[1]
  var t2d = x[3]

  //big terms
  var J2eff = (this.J2+(this.m2*pow(this.l2,2)*pow(cos(t2),2))/4)


  //now express the equations.
  var thresh = 100;
  // if(abs(tan(t2))>thresh){

  var t1dd = pow(this.J1+this.m2*this.l1*this.l2*this.J2/J2eff,-1)*(   tau - this.b1*t1d - m2*this.g*this.l1*this.J2/J2eff*tan(t2) - (2*this.l1*this.J2/(this.l2*cos(t2)*J2eff)+1)*this.b2*t2d +this.m2*this.g*this.l1*tan(t2) )
  var t2dd = pow(J2*(1+this.m2*this.l2*this.l2/this.J1)+this.m2*this.l2*this.l2/4*cos(t2)*cos(t2),-1)*(  kicktorque   -this.m2*this.l1*this.l2/(2*this.J1)*cos(t2)*tau + this.m2*this.g*this.l2/2*sin(t2)  - this.b2*(1+this.m2*this.l1*this.l1/this.J1)*t2d   -   this.m2*this.l1*this.l2/(2*this.J1)*cos(t2)*this.b1*t1d  +this.m2*this.m2*this.g*this.l1*this.l1*this.l2/(2*this.J1)*sin(t2)      )
  
  // var t2dd = 1/this.J2*(this.m2*this.g*this.l2/2*sin(t2) -.01*t2d)
  return [t1d,t1dd,t2d,t2dd]

}

}


//this display object works in PIXEL SPACE
function RPend_Display(ixorg,iyorg,r0,l0,r1,l1,r2,l2){
  this.xorg = ixorg;
  this.yorg = iyorg;
  this.l1 = l1;
  this.l2 = l2;
  this.l0=l0;
  this.r0=r0;
  this.r1 = r1;
  this.r2 = r2;
  
  this.draw = function(t1,t2){
    fill(255);
    stroke(0);
    cylinder(this.r0,this.l0);
    
    
    rotateX(PI/2);
    rotateZ(-t1);
    translate(0,-this.l1/2,this.l0/2);
    cylinder(this.r1,this.l1);
    rotateX(PI/2);
    rotateZ(t2);
    translate(0,+this.l2/2,this.l1/2);
    cylinder(this.r2,this.l2);

    //undo translations and rotations
    translate(0,-this.l2/2,-this.l1/2);
    rotateZ(-t2);
    rotateX(-PI/2);
    translate(0,+this.l1/2,-this.l0/2);
    rotateZ(t1);
    rotateX(-PI/2);
    
    
  }


}

function touchStarted() {
  Im_touched = true;
}

function touchEnded() {
  Im_touched = false;
}

function windowResized() {
  centerCanvas();
}

function centerCanvas() {
  var x = (windowWidth - width) / 2;
  var y = (windowHeight - height) / 2;
  cnv.position = [x, y];
}

function sign(x){
  if(x>=0){
    return 1
  }
  else{
    return -1
  }
}
