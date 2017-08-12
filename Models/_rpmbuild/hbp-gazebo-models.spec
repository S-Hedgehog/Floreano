Summary: The Gazebo models for the HBP NRP
Name: hbp-gazebo-models
Version: 1.2.1
Release: 0%{?dist}
BuildArch: noarch
License: GPL

%description
This package contains the Gazebo and gzweb models used for the HBP
Neurorobotics Platform.

%prep

%build

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/opt/hbp/gazebo
mkdir -p %{buildroot}/opt/gzweb
cp -r ../../tmp/models %{buildroot}/opt/hbp/gazebo
cp -r ../../tmp/assets %{buildroot}/opt/gzweb
chmod -R o+rx %{buildroot}/opt/hbp
chmod -R o+rx %{buildroot}/opt/gzweb

%files
/opt/hbp
/opt/gzweb
