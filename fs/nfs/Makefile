#
# Makefile for the Linux nfs filesystem routines.
#

obj-$(CONFIG_NFS_FS) += nfs.o

ifeq ($(CONFIG_ARM),y)
# Warnings are produced by the current arm cross compiler (v4.2.1) causing
# build abort. Version 4.3.1 of the compiler does not create this warning.
# So, remove the -Werror until the compiler can be updated for all build
# platforms to v4.3.1
CFLAGS_REMOVE_client.o = -Werror
endif

nfs-y 			:= client.o dir.o file.o getroot.o inode.o super.o nfs2xdr.o \
			   direct.o pagelist.o proc.o read.o symlink.o unlink.o \
			   write.o namespace.o mount_clnt.o
nfs-$(CONFIG_ROOT_NFS)	+= nfsroot.o
nfs-$(CONFIG_NFS_V3)	+= nfs3proc.o nfs3xdr.o
nfs-$(CONFIG_NFS_V3_ACL)	+= nfs3acl.o
nfs-$(CONFIG_NFS_V4)	+= nfs4proc.o nfs4xdr.o nfs4state.o nfs4renewd.o \
			   delegation.o idmap.o \
			   callback.o callback_xdr.o callback_proc.o \
			   nfs4namespace.o
nfs-$(CONFIG_SYSCTL) += sysctl.o
