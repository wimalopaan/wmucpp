# Dockerfile
FROM archlinux:base
RUN pacman -Syu --noconfirm

# Install misc packages
RUN pacman -S --noconfirm \
    7zip \
	arm-none-eabi-gcc \
	base-devel \
	bc \
	cpio \
	curl \
	dos2unix \
	erofs-utils \
	git \
	less \
	nano \
	python \
	python-pip \
	screen \
	tree \
	unzip \
	vim \
	wget \
	zip

# HACK: Allow base-devel to run with root user
RUN sed -i '/E_ROOT/d' /usr/bin/makepkg

# Install yay
RUN git clone https://aur.archlinux.org/yay-bin.git /tmp/yay-bin && \
    cd /tmp/yay-bin && \
	makepkg -si --noconfirm && \
	cd && rm -rf /tmp/yay-bin

# Personal dir dotfiles
#ADD home /home/

# Personal uploader scripts
#ADD root/bin /usr/local/bin/
