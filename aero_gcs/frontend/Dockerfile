# ============================================================================
#  AEGIS / AERO-GCS dashboard — FRONTEND image
#
#  This image contains ONLY the dashboard UI (a static React/Vite build served
#  by nginx). It is fully portable.
#
#  The UI talks to backend services that run NATIVELY on the operator's machine
#  (the browser calls them directly at localhost):
#     control_api   -> http://localhost:8000
#     rosbridge     -> ws://localhost:9090
#     video stream  -> http://localhost:8080
#  Those services orchestrate host ROS 2 / PX4 / Gazebo, so they are NOT in a
#  container (see aero_gcs/DOCKER.md).
#
#  Two stages: (1) build the SPA with Node, (2) ship only the static files on a
#  tiny nginx image. The Node toolchain never ends up in the final image.
# ============================================================================

# ---- stage 1: build the static site ----
FROM node:22-alpine AS build
WORKDIR /app

# Install deps first (cached until the lockfile changes).
COPY package.json package-lock.json ./
RUN npm ci

# Then build the app.
COPY . .
RUN npm run build          # -> /app/dist

# ---- stage 2: serve the static site ----
FROM nginx:alpine
# SPA routing: unknown paths fall back to index.html.
COPY nginx.conf /etc/nginx/conf.d/default.conf
COPY --from=build /app/dist /usr/share/nginx/html
EXPOSE 80
# nginx:alpine already runs `nginx -g 'daemon off;'` as its CMD.
