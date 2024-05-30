<template>
    <h1>Ponderada 3 : Vídeo no frontend direto do turtlebot</h1>

    <h4 class="text-center">Frontend em Vue com Vite</h4>
    <div class="control-container">
        <Control />
    </div>
    <div class="video-container">
        <Video></Video>
    </div>
</template>

<script setup lang="ts">
import {ref} from 'vue'
import Control from '../components/Control.vue';
import Video from '../components/Video.vue';

</script>

<style scoped>
.control-container {
  margin: 2%;
  position: fixed; 
  right: 0;        
  top: 50%;        
  transform: translateY(-50%); 
  display: flex;
  justify-content: center;
  align-items: center;
}

.video-container {
  margin: 2%;
  left: 20%;        
  transform: translateY(50%); 
  display: flex;
  justify-content: left;
  align-items: left;
}

</style>