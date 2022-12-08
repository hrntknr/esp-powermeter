FROM espressif/idf:release-v5.0
RUN apt update -y && \
  apt install -y locales zsh screen && \
  echo ". $IDF_PATH/export.sh" >>/etc/zsh/zprofile && \
  echo "export LANG=C.UTF-8" >>/etc/zsh/zprofile && \
  chsh -s /bin/zsh root