FROM malichao/carnd-term2-base:1.0.0

MAINTAINER Malcolm Ma <malichaooo@gmail.com>

# Arguments
ARG user
ARG uid
ARG gid
ARG home
ARG shell

RUN mkdir -p /home/${user} && \
    echo "${user}:x:${uid}:${gid}:${user},,,:/home/${user}:/bin/bash" >> /etc/passwd && \
    echo "${user}:x:${uid}:" >> /etc/group && \
    echo "${user} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${user} && \
    chmod 0440 /etc/sudoers.d/${user} && \
    chown ${uid}:${gid} -R /home/${user}

USER ${user}
ENV HOME /home/${user}

RUN echo ${shell}
