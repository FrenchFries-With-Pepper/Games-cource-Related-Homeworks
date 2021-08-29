#ifdef GL_ES
precision mediump float;
#endif

// Phong related variables
uniform sampler2D uSampler;
uniform vec3 uKd;
uniform vec3 uKs;
uniform vec3 uLightPos;
uniform vec3 uCameraPos;
uniform vec3 uLightIntensity;

varying highp vec2 vTextureCoord;//纹理坐标
varying highp vec3 vFragPos;//顶点位置
varying highp vec3 vNormal;//顶点法线

// Shadow map related variables
#define NUM_SAMPLES 50
#define BLOCKER_SEARCH_NUM_SAMPLES NUM_SAMPLES
#define PCF_NUM_SAMPLES NUM_SAMPLES
#define NUM_RINGS 10

#define EPS 1e-3
#define PI 3.141592653589793
#define PI2 6.283185307179586

// PCSS blocker size related variables
#define LIGHT_SIZE 0.03
#define NEAR 50.0
#define FAR 350.0
#define BIAS max(0.06 * (1.0 - dot(vNormal,normalize(uLightPos-vFragPos))),0.005);

uniform sampler2D uShadowMap;

varying vec4 vPositionFromLight;//光源视角下顶点的坐标

highp float rand_1to1(highp float x ) {
  // -1 -1
  return fract(sin(x)*10000.0);
}

highp float rand_2to1(vec2 uv ) {
  // 0 - 1
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract(sin(sn) * c);
}

float unpack(vec4 rgbaDepth) {
    const vec4 bitShift = vec4(1.0, 1.0/256.0, 1.0/(256.0*256.0), 1.0/(256.0*256.0*256.0));
    return dot(rgbaDepth, bitShift);
}

vec2 poissonDisk[NUM_SAMPLES];

void poissonDiskSamples( const in vec2 randomSeed ) {

  float ANGLE_STEP = PI2 * float( NUM_RINGS ) / float( NUM_SAMPLES );
  float INV_NUM_SAMPLES = 1.0 / float( NUM_SAMPLES );

  float angle = rand_2to1( randomSeed ) * PI2;
  float radius = INV_NUM_SAMPLES;
  float radiusStep = radius;

  for( int i = 0; i < NUM_SAMPLES; i ++ ) {
    poissonDisk[i] = vec2( cos( angle ), sin( angle ) ) * pow( radius, 0.75 );
    radius += radiusStep;
    angle += ANGLE_STEP;
  }
}

void uniformDiskSamples( const in vec2 randomSeed ) {

  float randNum = rand_2to1(randomSeed);
  float sampleX = rand_1to1( randNum ) ;
  float sampleY = rand_1to1( sampleX ) ;

  float angle = sampleX * PI2;
  float radius = sqrt(sampleY);

  for( int i = 0; i < NUM_SAMPLES; i ++ ) {
    poissonDisk[i] = vec2( radius * cos(angle) , radius * sin(angle)  );

    sampleX = rand_1to1( sampleY ) ;
    sampleY = rand_1to1( sampleX ) ;

    angle = sampleX * PI2;
    radius = sqrt(sampleY);
  }
}


float findBlocker( sampler2D shadowMap,  vec2 uv, float zReceiver, float lightSize ) {
  //get average blocker size
  float disToNear=zReceiver*(FAR-NEAR);//far - near = 300
  float disToLight=disToNear + NEAR;//near =50
  float blockSize=lightSize * disToNear / disToLight;
  // float blockSize=lightSize;
  //compute average blocker depth
  poissonDiskSamples(uv);
  float depth_in_shadow_map = 0.0;
  int sum=0;
  for(int i = 0; i<NUM_SAMPLES ; i++ ){
    vec2 coord=poissonDisk[i] * blockSize + uv;
    float depth = unpack(texture2D(shadowMap,coord));
    if(depth-0.0001<=zReceiver){//expel samples that z
      depth_in_shadow_map+=depth;
      sum++;
    }
  }
  depth_in_shadow_map /= float(sum);
	return depth_in_shadow_map;
  //  return 1.0;
}

float PCF(sampler2D shadowMap, vec4 shadowCoord,float filterSize) {
  vec2 shadowCoord_xy=shadowCoord.xy;
  float depth=shadowCoord.z-BIAS;//depth of shading point 
  poissonDiskSamples(shadowCoord_xy);
  float visibility=0.0;
  for(int i =0 ; i< NUM_SAMPLES ; i++){
    float depth_in_shadow_map = unpack(texture2D(shadowMap,poissonDisk[i]*filterSize+shadowCoord_xy));//depth in shadow map
    if(depth_in_shadow_map<0.001) depth_in_shadow_map=1.0;
    visibility += float(depth <= depth_in_shadow_map);
 }
  visibility /=  float(NUM_SAMPLES);
  return visibility;
}

float PCSS(sampler2D shadowMap, vec4 shadowCoords, float lightSize){
  // STEP 1: avgblocker depth
  vec2 uv=shadowCoords.xy;
  float dReceiver=shadowCoords.z;
  float dBlocker=findBlocker(shadowMap,uv,dReceiver, lightSize);
  float lightDis = dReceiver;

  // STEP 2: penumbra size
  float penumbra_size=float(lightDis - dBlocker) / dBlocker * lightSize;
  // STEP 3: filtering
  float visibility=PCF(shadowMap,shadowCoords,penumbra_size);
  return visibility;
  //return 1.0  ;

}


float useShadowMap(sampler2D shadowMap, vec4 shadowCoord){
  vec2 shadowCoord_xy=shadowCoord.xy;
  float depth_in_shadow_map=unpack(texture2D(shadowMap,shadowCoord.xy));

  float depth=shadowCoord.z-BIAS;
  return float(depth_in_shadow_map>=depth);
}

vec3 blinnPhong() {

  vec3 shadowCoord = vPositionFromLight.xyz*0.5+ 0.5;
  float visibility;
  // visibility = useShadowMap(uShadowMap,vec4(shadowCoord, 1.0));
  visibility = PCF(uShadowMap,vec4(shadowCoord,1.0), LIGHT_SIZE);
  // visibility = PCSS(uShadowMap,vec4(shadowCoord, 1.0),LIGHT_SIZE);
  // visibility = 1.0;

  vec3 color = texture2D(uSampler, vTextureCoord).rgb;
  color = pow(color, vec3(2.2));

  vec3 ambient = 0.015 * color;

  vec3 lightDir = normalize(uLightPos);
  vec3 normal = normalize(vNormal);
  float diff = max(dot(lightDir, normal), 0.0);
  vec3 light_atten_coff =
      uLightIntensity / pow(length(uLightPos - vFragPos), 2.0);
  vec3 diffuse = diff * light_atten_coff * color * visibility;

  vec3 viewDir = normalize(uCameraPos - vFragPos);
  vec3 halfDir = normalize((lightDir + viewDir));
  float spec = pow(max(dot(halfDir, normal), 0.0), 32.0);
  vec3 specular = uKs * light_atten_coff * spec * visibility;

  vec3 radiance = (ambient + diffuse + specular);
  vec3 phongColor = pow(radiance, vec3(1.0 / 2.2));
  return phongColor;
}

void main(void) {
  vec3 phongColor =  blinnPhong();
  gl_FragColor = vec4(phongColor, 1.0);
}