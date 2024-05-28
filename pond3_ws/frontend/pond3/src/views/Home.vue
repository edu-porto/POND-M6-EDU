<template>
    <h1>Ponderada 3 : Vídeo no frontend direto do turtlebot</h1>

    <h4 class="text-center">Frontend em Vue com Vite</h4>
    <div class="control-container">
        <Control />
    </div>
</template>

<script setup lang="ts">
import {ref} from 'vue'
import Control from '../components/Control.vue';
</script>

<style scoped>
.control-container {
  position: fixed; /* Position fixed to make it stay on the screen even when scrolling */
  right: 0;        /* Align to the right */
  top: 50%;        /* Align to the middle vertically */
  transform: translateY(-50%); /* Adjust to center the component vertically */
  display: flex;
  justify-content: center;
  align-items: center;
}

</style>