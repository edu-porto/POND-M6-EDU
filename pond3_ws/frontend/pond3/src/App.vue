<script setup lang="ts">
import Control from './components/Control.vue'
</script>

<template>
  <header>
    <div class="flex justify-center items-center w-full">
    </div>
    <h1>Ponderada 3 : Vídeo no frontend direto do turtlebot</h1>
  </header>
  <h4> Frontend em Vue com vite  </h4>
  <Control />
</template>

<style scoped>
.logo {
  height: 6em;
  padding: 1.5em;
  will-change: filter;
  transition: filter 300ms;
}
.logo:hover {
  filter: drop-shadow(0 0 2em #646cffaa);
}
.logo.vue:hover {
  filter: drop-shadow(0 0 2em #42b883aa);
}
</style>
